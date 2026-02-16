#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"

using namespace std::chrono_literals;

TEST_CASE("Test the initial pose service", "[odometry]")
{
    auto node = rclcpp::Node::make_shared("turtle_odom_test");

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create service client for initial pose service:
    auto client =
    node->create_client<nuturtle_control_interfaces::srv::InitialPose>("odometry/initial_pose");

    // Check to make sure the client is connected to the service:
    if (!client->wait_for_service(10s)) {
    FAIL("Service 'odometry/initial_pose' not available");
    }

    // Create the request with the desired initial pose:
    auto request = std::make_shared<nuturtle_control_interfaces::srv::InitialPose::Request>();
    request->pose.position.x = 1.0;         // Example initial request since we are just checking receipt.
    request->pose.position.y = 1.0;
    request->pose.orientation.z = 1.57;

    // Send the request asynchronously:
    auto future = client->async_send_request(request);

    // Spin node until response arrives or timeout
    auto start_time = rclcpp::Clock().now();
    bool received = false;

    while (!received && (rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION) {
    rclcpp::spin_some(node);
    if (future.wait_for(100ms) == std::future_status::ready) {
      received = true;
    }
    }

    // Require that the request was recieved:
    REQUIRE(received);
    REQUIRE(future.get()->success);
}

TEST_CASE("TF2_Listener for odom to base_footprint", "[odometry]")
{
    auto node = rclcpp::Node::make_shared("turtle_odom_test");

    // Declare test_duration parameter
    node->declare_parameter<double>("test_duration");
    const auto TEST_DURATION = node->get_parameter("test_duration").get_parameter_value().get<double>();

    // Create TF2 listener:
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Initialize variables to track received transform:
    bool transform_received = false;
    geometry_msgs::msg::TransformStamped received_transform;

    auto start_time = rclcpp::Clock().now();

    while (rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time).seconds() < TEST_DURATION))
  {
        // Try to lookup transform repeatedly until received
    if (!transform_received) {
      try {
        rclcpp::sleep_for(500ms);  // wait for TF to be published
        received_transform = tf_buffer.lookupTransform("odom", "blue/base_footprint",
          tf2::TimePointZero);
        transform_received = true;
      } catch (tf2::TransformException & ex) {
        // Ignore exceptions until transform is found
      }
    }

    rclcpp::spin_some(node);
    }

    // Require that the transform was recieved and that the appropriate frames are being published:
    REQUIRE(transform_received);
    CHECK(received_transform.header.frame_id == "odom");
    CHECK(received_transform.child_frame_id == "blue/base_footprint");
}
