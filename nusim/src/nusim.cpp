#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Nusimulator : public rclcpp::Node {
    public:
    Nusimulator() : Node("nusimulator") {
        // Declare the Parameter: 
        auto rate = this->declare_parameter<double>("rate", 100.0);
        // Declare the Publisher: 
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        // Create the timer callback:
        auto timer_callback = [this, rate]() -> void {
            // Print a Message Once:
            RCLCPP_INFO_ONCE(this->get_logger(), "The Timer rate is %f!", rate);
            // Publisher:
            auto message = std_msgs::msg::UInt64();
            message.data = timestep_++;
            RCLCPP_INFO(this->get_logger(), "Timestep: '%lu'", message.data);
            this->publisher_->publish(message);
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
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
        uint64_t timestep_;
}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}