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
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"

/// \brief A class to launch a Simulator Node
class Odometry : public rclcpp::Node {
public:
    Odometry()
    : Node("odometry") {
        // Initialize the Parameters:
        body_id_ = declare_parameter<std::string>("body_id", "base_footprint");
        odom_id_ = declare_parameter<std::string>("odom_id", "odom");
        wheel_left_ = declare_parameter<std::string>("wheel_left");
        wheel_right_ = declare_parameter<std::string>("wheel_right");
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
        track_width_ = declare_parameter<double>("track_width", 0.033);
        
        // Check required parameters:
        if (wheel_left_.empty()) {
            RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameter 'wheel_left' is required but not set. Exiting...");
            rclcpp::shutdown();  // terminate the node
            return;
        }
        if (wheel_right_.empty()) {
            RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameter 'wheel_right' is required but not set. Exiting...");
            rclcpp::shutdown(); // terminate the node
            return;
        }

        // Initialize the DiffDrive model for internal odometry state:
        diff_drive_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Construct the publisher for odometry:
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Construct the subscriber for joint states:
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Update the internal odemetry state every time a new joint state message is received, and publish the current odometry message and transform:            
            // Find indices of wheel joints in joint state message and set a map:
            static std::unordered_map<std::string, size_t> joint_map;
            if (joint_map.empty()) {
                for (size_t i = 0; i < msg->name.size(); i++) {
                    // Check to see if joint name matches:
                    joint_map[msg->name[i]] = i;
                }
            }
            // Current wheel positions:
            double phi_left  = msg->position[joint_map[wheel_left_]];
            double phi_right = msg->position[joint_map[wheel_right_]];

            // Time and position change since last message:
            double dt = std::clamp((msg->header.stamp - prev_time_).seconds(), 1e-9, 1.0); // Clamp dt to avoid negatives and large start up values.

            // Update forward kinematics:
            diff_drive_.update_fk(phi_left, phi_right);

            // Get the Body-frame twist:
            auto delta_x_body = diff_drive_.get_q().translation().x - prev_pose_x_;
            auto delta_y_body = 0.0; // diff drive cannot move sideways, so we assume no change in y position in the body frame.
            auto delta_theta_body = turtlelib::normalize_angle(diff_drive_.get_q().rotation() - prev_pose_theta_);
            // Set body-frame twist:
            turtlelib::Twist2D twist_body{turtlelib::Vector2D{delta_x_body / dt, delta_y_body / dt}, delta_theta_body / dt};

            // Transform and Integrate to world-frame pose:
            turtlelib::Twist2D delta_twist_world; 
            delta_twist_world.x = twist_body.x * std::cos(prev_pose_theta_) - twist_body.y * std::sin(prev_pose_theta_);
            delta_twist_world.y = twist_body.x * std::sin(prev_pose_theta_) + twist_body.y * std::cos(prev_pose_theta_);
            delta_twist_world.omega = twist_body.omega;
            // Integrtate to get the change in pose in the world frame:
            turtlelib::Transform2D delta_pose_world = turtlelib::integrate_twist(delta_twist_world);

            // Get the new pose in the world frame by applying the change in pose to the previous pose:
            double pose_x = prev_pose_x_ + delta_pose_world.translation().x;
            double pose_y = prev_pose_y_ + delta_pose_world.translation().y;
            double pose_theta = turtlelib::normalize_angle(prev_pose_theta_ + delta_pose_world.rotation());

            // Update the internal odometry state:
            prev_phi_left_ = phi_left;
            prev_phi_right_ = phi_right;
            prev_pose_x_ = pose_x;
            prev_pose_y_ = pose_y;
            prev_pose_theta_ = pose_theta;
            prev_time_ = msg->header.stamp;

            // ------------------------------------------------------------------------------
            // Build and publish Odometry and TF:
            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = msg->header.stamp;
            odometry_msg.header.frame_id = odom_id_;
            odometry_msg.child_frame_id = body_id_;

            // Set pose in world frame:
            odometry_msg.PoseWithCovariance.pose.position.x = pose_x;
            odometry_msg.PoseWithCovariance.pose.position.y = pose_y;
            odometry_msg.PoseWithCovariance.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, pose_theta));
            odometry_msg.PoseWithCovariance.covariance = {0.0}; // Set covariance to 0.

            // Set the Linear and angular velocity relative to body frame:
            // Get the current twist from the DiffDrive model:
            odometry_msg.TwistWithCovariance.twist.linear.x = twist_body.x;
            odometry_msg.TwistWithCovariance.twist.linear.y = twist_body.y;
            odometry_msg.TwistWithCovariance.twist.angular.z = twist_body.omega;
            odometry_msg.TwistWithCovariance.covariance = {0.0}; // Set covariance to 0.
            // Publish the odometry message:
            odometry_publisher_->publish(odometry_msg);

            // Publish the odometry transform:
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = msg->header.stamp;
            t.header.frame_id = odom_id_;
            t.child_frame_id = body_id_;
            t.transform.translation.x = pose_x;
            t.transform.translation.y = pose_y;
            t.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, pose_theta);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        });

        // Initialize Reset service
        reset_service_ = this->create_service<nuturtle_control_interfaces::srv::InitialPose>(
                "~/initial_pose",
                std::bind(&Nusimulator::handle_service_initial_pose, this, std::placeholders::_1, std::placeholders::_2)
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
        const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
        const std::shared_ptr<nuturtle_control::srv::InitialPose::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Initial Pose Reset!");
        // Reset the internal odometry state with the requested configuration:
        prev_pose_x_ = request->pose.position.x;
        prev_pose_y_ = request->pose.position.y;
        prev_pose_theta_ = tf2::getYaw(request->pose.orientation); // These is orienation about z in 2D.
        prev_time_ = this->get_clock()->now(); // Reset the time to now at
        
        // Return success for reset:
        response->success = true;
    }

    // Initialize ROS 2 Infrustructure:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // Initialize the parameters:
    std::string body_id_;
    std::string odom_id_;
    std::string wheel_left_;
    std::string wheel_right_;
    double wheel_radius_;
    double track_width_;

    // Internal odometry state:
    turtlelib::DiffDrive diff_drive_;
    double prev_phi_left_ = 0.0;
    double prev_phi_right_ = 0.0;
    double prev_pose_x_ = 0.0;
    double prev_pose_y_ = 0.0;
    double prev_pose_theta_ = 0.0;
    rclcpp::Time prev_time_ = this->get_clock()->now();     // Set the time to now at init.
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
