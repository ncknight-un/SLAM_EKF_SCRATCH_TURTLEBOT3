#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

using namespace std::chrono_literals;

// ################################### Begin_Citation [9] ###################################
// The following test case is adapted from the example test case provided in the Catch2 documentation:
TEST_CASE("cmd_vel pure translation equal correct wheel_cmd", "[turtle_control]") 
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create publisher fro cmd_vel:
    auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    bool msg_received = false;
    nuturtlebot_msgs::msg::WheelCommands received_msg;

    auto wheel_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10,
      [&](nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
          received_msg = *msg;
          msg_received = true;
      }
    );

    auto start_time = rclcpp::Clock().now();

    while (rclcpp::ok() &&
           ((rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION))
    {
        // Publish command repeatedly until received
        if (!msg_received) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 1.0;
            twist.angular.z = 0.0;
            cmd_pub->publish(twist);
        }

        rclcpp::spin_some(node);
    }

    REQUIRE(msg_received);
    CHECK(received_msg.left_velocity == received_msg.right_velocity);
    CHECK(received_msg.left_velocity != 0);
}
// ################################### End_Citation [9] ###################################
