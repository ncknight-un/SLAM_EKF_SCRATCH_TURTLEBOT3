/// \file
/// \brief Creates a ROS 2 Node which enables control of the turtlebot via geometry/msg/Twist messages on the cmd_vel topic.
///
/// PARAMETERS:
///     wheel_radius (double): The radius of the robot's wheels in meters
///     track_width (double): The distance between the robot's wheels in meters
///     motor_cmd_max (int): The maximum motor command that can be sent to the robot
///     motor_cmd_per_rad_sec (double): The conversion factor from wheel angular velocity in radians per second to motor command units
///     encode_ticks_per_rad (int): The number of encoder ticks per radian of wheel rotation
/// PUBLISHES:
///     ~ wheel_cmd (nuturtle_control/msg/WheelCommands): The motor commands for the left and right wheels, converted from the cmd_vel topic
///     ~ joint_states (sensor_msgs/msg/JointState): The current joint states of the
///     ~ path (nav_msgs/msg/Path): The current path of the robot based on its position updates for odometry.
/// SUBSCRIBES:
///     ~ cmd_vel (geometry_msgs/msg/Twist): The desired linear and angular velocity of the robot base, which is converted to wheel commands and published on the wheel_cmd topic
///     ~ sensor_data (nuturtle_control/msg/SensorData): The current wheel encoder ticks, which is used to update the DiffDrive model and publish the current joint states on the joint_states
/// SERVERS:
///     None
/// CLIENTS:
///     None
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

/// \brief A class to launch a Simulator Node
class TurtleControl : public rclcpp::Node {
public:
  TurtleControl()
  : Node("turtle_control")
  {
        // Check to make sure  that all parameters are valid:
    if (wheel_radius_ <= 0 || track_width_ <= 0 || motor_cmd_max_ <= 0 ||
      motor_cmd_per_rad_sec_ <= 0 || encode_ticks_per_rad_ <= 0)
    {
            // Log an error message and throw an exception if any parameter is invalid
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Not all parameters were initialized:" <<
            " wheel_radius=" << wheel_radius_ << ", track_width=" << track_width_ <<
        ", motor_cmd_max=" << motor_cmd_max_
                                                                                          <<
        ", motor_cmd_per_rad_sec=" << motor_cmd_per_rad_sec_ << ", encode_ticks_per_rad=" <<
        encode_ticks_per_rad_);
      rclcpp::shutdown();        // terminate the node
    }
        // Construct the publisher for wheel commands:
    wheel_cmd_publisher_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd",
      10);
        // Construct the publisher for joint states:
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
      10);

      // Construct the publisher for the robot path:
      path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("blue/odom_path", 10);

        // Construct the subscriber and publisher for cmd_vel:
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            // Convert the Twist message to wheel commands:
          turtlelib::Twist2D twist{msg->angular.z, msg->linear.x, 0.0};
          turtlelib::wheel_vel wheel_vel = this->diff_drive_.compute_ik(twist);

            // Convert the wheel speeds to motor commands and clamp them to the maximum allowed value:
          auto left_motor_cmd = std::clamp(static_cast<int>(wheel_vel.v_lw *
          this->motor_cmd_per_rad_sec_), -this->motor_cmd_max_, this->motor_cmd_max_);
          auto right_motor_cmd = std::clamp(static_cast<int>(wheel_vel.v_rw *
          this->motor_cmd_per_rad_sec_), -this->motor_cmd_max_, this->motor_cmd_max_);

            // Publish the wheel commands to the wheel_cmd topic:
          nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
          wheel_cmd_msg.left_velocity = left_motor_cmd;
          wheel_cmd_msg.right_velocity = right_motor_cmd;
          wheel_cmd_publisher_->publish(wheel_cmd_msg);
        });

        // Construct the subscriber for sensor data to joint state publisher:
    sensor_data_subscriber_ =
      this->create_subscription<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10,
        [this](const nuturtlebot_msgs::msg::SensorData::SharedPtr msg) {
            // Calculate the change in wheel positions from encoder ticks:
          auto phi_left = static_cast<double>(msg->left_encoder) / this->encode_ticks_per_rad_;
          auto phi_right = static_cast<double>(msg->right_encoder) / this->encode_ticks_per_rad_;

            // Caluate dt:
          if (prev_time_.nanoseconds() == 0) {
            prev_time_ = msg->stamp;
            prev_phi_left_ = phi_left;
            prev_phi_right_ = phi_right;
            return;
          }
          curr_time_ = msg->stamp;
          rclcpp::Duration delta = curr_time_ - prev_time_;
          double dt = delta.seconds();

            // Error control for dt of zero.
          if (dt <= 0.0) {
            return;
          }

          auto delta_phi_left = turtlelib::normalize_angle(phi_left - prev_phi_left_);
          auto delta_phi_right = turtlelib::normalize_angle(phi_right - prev_phi_right_);
          auto dphi_left = delta_phi_left / dt;
          auto dphi_right = delta_phi_right / dt;

            // Update the DiffDrive model with new wheel positions:
          diff_drive_.update_fk(phi_left, phi_right);

            // Update internal tracking:
          prev_time_ = curr_time_;
          prev_phi_left_ = phi_left;
          prev_phi_right_ = phi_right;

            // Publish the joint state for the left and right wheel joints:
          sensor_msgs::msg::JointState joint_state_msg;
          joint_state_msg.header.stamp = msg->stamp;
            // Set joint positions:
          joint_state_msg.name = {"wheel_left_joint", "wheel_right_joint"};
          joint_state_msg.position = {phi_left, phi_right};
          joint_state_msg.velocity = {dphi_left, dphi_right};
          joint_state_publisher_->publish(joint_state_msg);

          // Publish the robot path for odometry:
          robot_path_.header.stamp = msg->stamp;  // Keep the same timestamp as sensor data update
          robot_path_.header.frame_id = "nusim/world";
          geometry_msgs::msg::PoseStamped pose;
          pose.header = robot_path_.header;
          // Update the pose of the robot:
          pose.pose.position.x = diff_drive_.get_q().translation().x;
          pose.pose.position.y = diff_drive_.get_q().translation().y;
          pose.pose.position.z = 0.0;
          // Orienation of the robot as a quaternion:
          tf2::Quaternion q;
          q.setRPY(0, 0, diff_drive_.get_q().rotation());
          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();
          robot_path_.poses.push_back(pose);
          path_publisher_->publish(robot_path_);
        });
  }

private:
    // Initialize ROS 2 Infrustructure:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    // Initialize the parameters:
  double wheel_radius_ = declare_parameter<double>("wheel_radius");
  double track_width_ = declare_parameter<double>("track_width");
  int motor_cmd_max_ = declare_parameter<int>("motor_cmd_max");
  double motor_cmd_per_rad_sec_ = declare_parameter<double>("motor_cmd_per_rad_sec");
  int encode_ticks_per_rad_ = declare_parameter<int>("encode_ticks_per_rad");

    // Internal Tracking for dt positioning:
  rclcpp::Time prev_time_;
  rclcpp::Time curr_time_;
  double prev_phi_left_ = 0.0;
  double prev_phi_right_ = 0.0;
    // Initialize the DiffDrive model:
  turtlelib::DiffDrive diff_drive_{track_width_, wheel_radius_};

  // Track the robot path for odometry:
  nav_msgs::msg::Path robot_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
