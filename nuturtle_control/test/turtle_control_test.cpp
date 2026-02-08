#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

using namespace std::chrono_literals;

// ################################### Begin_Citation [9] ###################################
// The following test case is adapted from the example test case provided in the Catch2 documentation:
TEST_CASE("cmd_vel pure translation equal correct wheel_cmd", "[turtle_control]") 
{
  auto node = rclcpp::Node::make_shared("turtle_control_test");

  // Publisher to cmd_vel
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Variables to capture output
  nuturtlebot_msgs::msg::WheelCommands received_msg;
  bool msg_received = false;

  // Subscriber to wheel_cmd
  auto wheel_sub = node->create_subscription<
    nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 
      10,
      [&](nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
      {
        received_msg = *msg;
        msg_received = true;
      });

  // Give time for connections
  rclcpp::sleep_for(200ms);

  // Publish pure translation command
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;   // forward motion
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;  // no rotation

  cmd_pub->publish(twist);

  // Spin for a short time to process callbacks
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (!msg_received && (rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2.0)) {
    rclcpp::spin_some(node);
  }

  // Assertions
  REQUIRE(msg_received);
  CHECK(received_msg.left_velocity == received_msg.right_velocity);
  CHECK(received_msg.left_velocity != 0);
}
// ################################### End_Citation [9] ###################################
