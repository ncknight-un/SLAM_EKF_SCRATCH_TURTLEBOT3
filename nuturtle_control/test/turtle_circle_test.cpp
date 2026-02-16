#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control_interfaces/srv/control.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

TEST_CASE("Test that the circle published cmd_vel at the correct frequency", "[circle]")
{
    auto node = rclcpp::Node::make_shared("turtle_circle_test");
    auto count = 0;

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create service client for circle control service:
    auto client = node->create_client<nuturtle_control_interfaces::srv::Control>("circle/control");

    // Check to make sure the client is connected to the service:
    if (!client->wait_for_service(10s)) {
    FAIL("Service 'circle/control' not available");
    }

    // Create a service to listen to cmd_vel messages published by the circle node:
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [&](geometry_msgs::msg::Twist::SharedPtr msg) {
        // Count how many messages are received in the test duration:
        count++;
        (void)msg; // Avoid unused variable warning since we only care about count for this test, not the content of the message.
    });

    // Create the request with the desired control params:
    auto request = std::make_shared<nuturtle_control_interfaces::srv::Control::Request>();
    request->velocity = 0.5;
    request->radius = 0.5;

    // Send the request asynchronously:
    auto future = client->async_send_request(request);

    // Spin node until response arrives or timeout
    auto start_time = rclcpp::Clock().now();
    bool received = false;

    // Keep spinning for full test duration:
    while ((rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION) {
    rclcpp::spin_some(node);      // process incoming messages
    received = true;     // We got a message successfully
    }

    // Require that the frequency of cmd_vel during test duration is approximately 100Hz.
    REQUIRE(received);
    CHECK(count > (TEST_DURATION * 95)); // Check that at least 95%
}
