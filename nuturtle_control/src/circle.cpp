/// \file
/// \brief Creates a ROS 2 Node which publishes cmd_vel commands to cause the robot to drive in a circle of a specified radius at a specified speed.
///
/// PARAMETERS:
///    frequency (int): Timer frequency for the main control loop (Hz)
/// PUBLISHES:
///     ~ /cmd_vel (geometry_msgs/msg/Twist): The desired linear and angular velocity of the robot base, which is converted to wheel commands and published on the wheel_cmd topic
/// SUBSCRIBES:
///     ~ 
/// SERVERS:
///     ~ control (nuturtle::srv::control): Sets the control of the node. Takes a velocity and radius and causes the robot to drive in a circle of the specified radius at the specified velocity.
///     ~ reverse (std_srvs::srv::Empty): Reverses the direction of the circle drive. 
///     ~ stop (std_srvs::srv::Empty): Stops the robot from moving.
/// CLIENTS:
///     None
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control_interfaces/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

/// \brief A class to launch a Simulator Node
class Circle : public rclcpp::Node {
public:
    Circle()
    : Node("circle") {
        // Initialize the Parameters:
        auto frequency = declare_parameter<int>("frequency", 100);

        // Construct the publisher for cmd_vel:
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
    }

    // Create the timer callback:
    auto timer_callback = [this, rate]() -> void {
        // Print a Message Once:
        RCLCPP_INFO_ONCE(this->get_logger(), "The Timer rate is %f!", rate);

        // Publish the cmd_vel message:
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.0;
        cmd_vel_publisher_->publish(message);
    };

    // Set the timer:
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
      timer_callback);

    // Control service
    control_service_ = this->create_service<nuturtle_control_interfaces::srv::Control>(
            "~/control",
            std::bind(&Circle::handle_service_control, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Reverse service
    reverse_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reverse",
            std::bind(&Circle::handle_service_reverse, this, std::placeholders::_1, std::placeholders::_2)
    );
private:
    // Initialize ROS 2 Infrustructure:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
