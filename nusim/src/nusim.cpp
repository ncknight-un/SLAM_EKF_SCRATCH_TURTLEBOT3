#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class Nusimulator : public rclcpp::Node {
    public:
    Nusimulator() : Node("nusimulator") {
        // Declare the Parameter: 
        auto rate = this->declare_parameter<double>("rate", 100.0);
        // Declare the Publisher: 
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create the timer callback:
        auto timer_callback = [this, rate]() -> void {
            // Print a Message Once:
            RCLCPP_INFO_ONCE(this->get_logger(), "The Timer rate is %f!", rate);
            // Publisher:
            auto message = std_msgs::msg::UInt64();
            message.data = timestep_++;
            RCLCPP_DEBUG(this->get_logger(), "Timestep: '%lu'", message.data);
            this->publisher_->publish(message);

            // Assign parameters to corresponding tf variables and broadcast:
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "nusim/world";
            t.child_frame_id = "red/base_footprint";
            // Turtlebot only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = x0_;
            t.transform.translation.y = y0_;
            t.transform.translation.z = 0.0;

            // For the same reason, turtlebot can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            tf2::Quaternion q;
            q.setRPY(0, 0, theta0_);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        };
        // Set the timer:
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate)), timer_callback);
        // Reset service
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&Nusimulator::handle_service_reset, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
    
    private:
        void handle_service_reset(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                (void)request;
                (void)response;
                RCLCPP_INFO(this->get_logger(),"Timestep Reset!");
                timestep_ = 0;      // Reset the timestep counter
                // ####################### Begin_Citation [7] ############################
                // Reset the simulation positions:
                x0_ = this->get_parameter("x0").as_double();
                y0_ = this->get_parameter("y0").as_double();
                theta0_ = this->get_parameter("theta0").as_double();
                // ######################## End_Citation [7] ###############################
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        uint64_t timestep_;
        // Set the Ground Truth values:
        double x0_ = this->declare_parameter<double>("x0", 0.0);
        double y0_ = this->declare_parameter<double>("y0", 0.0);
        double theta0_ = this->declare_parameter<double>("theta0", 0.0);
}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}