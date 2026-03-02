/// \file
/// \brief Creates a ROS 2 Node which publishes odimetry messages and the odometry transform.
///
/// PARAMETERS:
///     body_id (string): The name of the robot's body frame to use in the odometry transform.
///     odom_id (string): The name of the odometry frame to use in the odometry transform.
///     wheel_left (string): The name of the left wheel joint to read from the joint_states topic.
///     wheel_right (string): The name of the right wheel joint to read from the joint_states topic.
///     wheel_radius (double): The radius of the robot's wheels in meters, used to compute odometry.
///     track_width (double): The distance between the robot's wheels in meters, used to compute odometry.
/// PUBLISHES:
///     ~ odom (nav_msgs/msg/Odometry): The current estimated pose and velocity of the robot base in the odom frame.
/// SUBSCRIBES:
///     ~ joint_states (sensor_msgs/msg/JointState): The current joint states of the robot, which is used to update the internal odometry state and publish the current odometry message and transform.
/// SERVERS:
///     initial pose - Resets the robots odometry to think it is at the requested configuration.
/// CLIENTS:
///     None
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "std_srvs/srv/empty.hpp"

/// \brief A class to launch a Simulator Node
class Odometry : public rclcpp::Node {
public:
  Odometry()
  : Node("slam_odom")
  {
        // Check required parameters:
    if (wheel_left_.empty()) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
        "Parameter 'wheel_left' is required but not set. Exiting...");
      rclcpp::shutdown();        // terminate the node
      return;
    }
    if (wheel_right_.empty()) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
        "Parameter 'wheel_right' is required but not set. Exiting...");
      rclcpp::shutdown();       // terminate the node
      return;
    }

        // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Construct the publisher for odometry:
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Construct the subscriber for joint states:
    joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Update the internal odemetry state every time a new joint state message is received, and publish the current odometry message and transform:
            // Find indices of wheel joints in joint state message and set a map:
          static std::unordered_map<std::string, size_t> joint_map;
          if (joint_map.empty()) {
            for (size_t i = 0; i < msg->name.size(); i++) {
                    // Check to see if joint name matches:
              joint_map[msg->name[i]] = i;
            }
          }
          double phi_left = msg->position[joint_map[wheel_left_]];
          double phi_right = msg->position[joint_map[wheel_right_]];

            // Compute Twist
          turtlelib::Twist2D tw = diff_drive_.compute_twist(turtlelib::Vector2D{phi_left,
            phi_right});

            // Update forward kinematics:
          diff_drive_.update_fk(phi_left, phi_right);

            // ------------------------------------------------------------------------------
            // Build and publish Odometry and TF:
          nav_msgs::msg::Odometry odometry_msg;
          odometry_msg.header.stamp = msg->header.stamp;
          odometry_msg.header.frame_id = odom_id_;
          odometry_msg.child_frame_id = body_id_;

            // Set pose in body frame:
          odometry_msg.pose.pose.position.x = diff_drive_.get_q().translation().x;
          odometry_msg.pose.pose.position.y = diff_drive_.get_q().translation().y;
          odometry_msg.pose.pose.position.z = 0.0;
          tf2::Quaternion q;
          q.setRPY(0, 0, diff_drive_.get_q().rotation());
          odometry_msg.pose.pose.orientation.x = q.x();
          odometry_msg.pose.pose.orientation.y = q.y();
          odometry_msg.pose.pose.orientation.z = q.z();
          odometry_msg.pose.pose.orientation.w = q.w();
          odometry_msg.pose.covariance = {0.0};   // Set covariance to 0.

            // Set the Linear and angular velocity relative to body frame:
            // Get the current twist from the DiffDrive model:
          odometry_msg.twist.twist.linear.x = tw.x;
          odometry_msg.twist.twist.linear.y = tw.y;
          odometry_msg.twist.twist.angular.z = tw.omega;
          odometry_msg.twist.covariance = {0.0};   // Set covariance to 0.
            // Publish the odometry message:
          odometry_publisher_->publish(odometry_msg);

            // Publish the odometry transform:
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = msg->header.stamp;
          t.header.frame_id = odom_id_;
          t.child_frame_id = body_id_;
          t.transform.translation.x = odometry_msg.pose.pose.position.x;
          t.transform.translation.y = odometry_msg.pose.pose.position.y;
          t.transform.translation.z = 0.0;
          t.transform.rotation.x = odometry_msg.pose.pose.orientation.x;
          t.transform.rotation.y = odometry_msg.pose.pose.orientation.y;
          t.transform.rotation.z = odometry_msg.pose.pose.orientation.z;
          t.transform.rotation.w = odometry_msg.pose.pose.orientation.w;
          tf_broadcaster_->sendTransform(t);
        });

        // Initialize Reset service
    reset_service_ = this->create_service<nuturtle_control_interfaces::srv::InitialPose>(
                "~/initial_pose",
                std::bind(&Odometry::handle_service_initial_pose, this, std::placeholders::_1,
      std::placeholders::_2)
    );
  }

private:
    /// \brief Resets the odometry to think the robot is at the requested configuration.
    ///
    /// Resets the internal odometry state to make the robot think it is at the configuration specified by service call.
    ///
    /// \param request - The requested initial pose for the robot to reset to, in the form of a geometry_msgs/Pose message.
    /// \param response - bool success value indicating whether the reset was successful.
    /// \returns void
  void handle_service_initial_pose(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Initial Pose Reset!");
        // Reset the internal odometry state with the requested configuration:
    diff_drive_.set_q(turtlelib::Transform2D(turtlelib::Vector2D{request->pose.position.x,
        request->pose.position.y}, request->pose.orientation.z));
        // Return success for reset:
    response->success = true;
  }

    // Initialize ROS 2 Infrustructure:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Initialize the Parameters:
  std::string body_id_ = declare_parameter<std::string>("body_id", "blue/base_footprint");
  std::string odom_id_ = declare_parameter<std::string>("odom_id", "odom");
  std::string wheel_left_ = declare_parameter<std::string>("wheel_left");
  std::string wheel_right_ = declare_parameter<std::string>("wheel_right");
  double wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
  double track_width_ = declare_parameter<double>("track_width", 0.16);

    // Initialize the DiffDrive model for internal odometry state:
  turtlelib::DiffDrive diff_drive_{track_width_, wheel_radius_};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
