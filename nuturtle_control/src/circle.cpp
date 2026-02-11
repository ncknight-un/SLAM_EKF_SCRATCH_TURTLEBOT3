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
#include "nuturtle_control_interfaces/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief A class to launch a Simulator Node
class Circle : public rclcpp::Node {
public:
  Circle()
  : Node("circle")
  {
        // Declare the parameters:
    declare_parameter<int>("frequency", 10);
        // Get the Parameters:
    this->get_parameter("frequency", frequency_);

        // Construct the publisher for cmd_vel:
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create the timer callback:
    auto timer_callback = [this]() -> void {
            // Print a Message Once:
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Timer rate is " << frequency_ << "!");

            // Publish the cmd_vel message:
        auto message = geometry_msgs::msg::Twist();
            // Check to make sure that the radius is not zero:
        if (radius_ == 0.0) {
          RCLCPP_WARN(this->get_logger(),
          "Radius was set to zero, drive in a circle not possible! Setting velocity to 0.0.");
          velocity_ = 0.0;
                // Note: Counter clockwise is positive and clockwise is negative.
          message.angular.z = 0.0;
        } else {
                // Note: Counter clockwise is positive and clockwise is negative.
          message.angular.z = velocity_ / radius_;
        }
            // linear velocity speed in x direction for robot body frame.
        message.linear.x = velocity_;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;

        cmd_vel_publisher_->publish(message);
      };

        // Set the timer:
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / frequency_)),
        timer_callback);

        // Control service
    control_service_ = this->create_service<nuturtle_control_interfaces::srv::Control>(
                "~/control",
                std::bind(&Circle::handle_service_control, this, std::placeholders::_1,
      std::placeholders::_2)
    );
        // Reverse service
    reverse_service_ = this->create_service<std_srvs::srv::Empty>(
                "~/reverse",
                std::bind(&Circle::handle_service_reverse, this, std::placeholders::_1,
      std::placeholders::_2)
    );
        // Stop service
    stop_service_ = this->create_service<std_srvs::srv::Empty>(
                "~/stop",
                std::bind(&Circle::handle_service_stop, this, std::placeholders::_1,
      std::placeholders::_2)
    );
  }

private:
    // Control service callback
  void handle_service_control(
    const std::shared_ptr<nuturtle_control_interfaces::srv::Control::Request> request,
    const std::shared_ptr<nuturtle_control_interfaces::srv::Control::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "New Circle Control Parameters Set!");
        // Set the velocity and radius parameters to the values specified in the service request
    velocity_ = request->velocity;
    radius_ = request->radius;
        // Set the response to indicate success
    response->success = true;
  }
    // Reverse service callback
  void handle_service_reverse(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
        // Reverse the direction of the circle drive by negating the velocity parameter
    RCLCPP_INFO(this->get_logger(), "Circle Direction Reversed!");
    velocity_ = -velocity_;
  }
    // Stop service callback
  void handle_service_stop(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
    RCLCPP_INFO(this->get_logger(), "Circle Path Stopped!");
        // Stop the robot from moving by setting the velocity parameter to 0.0:
    velocity_ = 0.0;
  }

    // Initialize ROS 2 Infrustructure:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<nuturtle_control_interfaces::srv::Control>::SharedPtr control_service_;

    // Initialize the Parameters:
  int frequency_;
  double velocity_ = 0.0;
  double radius_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
