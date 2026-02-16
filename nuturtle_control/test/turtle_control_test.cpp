#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

    auto wheel_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd",
    10,
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

TEST_CASE("cmd_vel pure rotation to correct wheel_cmd", "[turtle_control]")
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create publisher to cmd_vel:
    auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    bool msg_received = false;
    nuturtlebot_msgs::msg::WheelCommands received_msg;

    auto wheel_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd",
    10,
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
      twist.linear.x = 0.0;
      twist.angular.z = 1.0;
      cmd_pub->publish(twist);
    }

    rclcpp::spin_some(node);
    }

    REQUIRE(msg_received);
    CHECK(received_msg.left_velocity == -received_msg.right_velocity);
    CHECK(received_msg.left_velocity != 0);
}


TEST_CASE("Encoder data on sensors is successfully converted to joint states", "[turtle_control]")
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create publisher to cmd_vel:
    auto sensor_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

    // Initialize variables to track received message:
    bool msg_received = false;
    sensor_msgs::msg::JointState received_msg;

    auto wheel_sub = node->create_subscription<sensor_msgs::msg::JointState>("joint_states",
    10,
      [&](sensor_msgs::msg::JointState::SharedPtr msg) {
        received_msg = *msg;
        msg_received = true;
      }
    );

    auto start_time = rclcpp::Clock().now();

    while (rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION))
  {
        // Publish command repeatedly until received for sensor data:
    if (!msg_received) {
      nuturtlebot_msgs::msg::SensorData sensor_data_msg;
      sensor_data_msg.left_encoder = 100;
      sensor_data_msg.right_encoder = 100;
      sensor_data_msg.stamp = rclcpp::Clock().now();
      sensor_pub->publish(sensor_data_msg);
    }

    rclcpp::spin_some(node);
    }

    REQUIRE(msg_received);
    REQUIRE_THAT(received_msg.position[0], Catch::Matchers::WithinRel(0.15337, 0.001)); // Check that left wheel position is updated
    REQUIRE_THAT(received_msg.position[1], Catch::Matchers::WithinRel(0.15337, 0.001)); // Check that right wheel position is updated
}
