/// \file
/// \brief Creates a ROS 2 Node which internally tracks the Ground Truth of a Turtlebot in simulation.
///
/// PARAMETERS:
///     rate (double): Timer frequency for the main simulation loop (Hz)
///     robot.x0 (double): Initial x-coordinate of the robot in the world frame
///     robot.y0 (double): Initial y-coordinate of the robot in the world frame
///     robot.theta0 (double): Initial orientation (yaw) of the robot in radians
///     arena_x_length (double): Length of the arena in the x-direction (meters)
///     arena_y_length (double): Length of the arena in the y-direction (meters)
///     arena_thickness (double): Thickness of arena walls (meters)
///     obstacles_x (vector<double>): X-coordinates of obstacles
///     obstacles_y (vector<double>): Y-coordinates of obstacles
///     obstacles_r (vector<double>): Radii of obstacles
///     obstacle_height (double): Height of obstacles (meters)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::UInt64): Current simulation timestep counter
///     ~/real_walls (visualization_msgs::msg::MarkerArray): Markers representing arena walls
///     ~/real_obstacles (visualization_msgs::msg::MarkerArray): Markers representing obstacles
/// SUBSCRIBES:
///     ~/cmd_vel (geometry_msgs::msg::Twist): Velocity commands for the robot
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): Resets the simulation timestep and robot initial pose
/// CLIENTS:
///     None
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/// \brief A class to launch a Simulator Node
class Nusimulator : public rclcpp::Node {
public:
  Nusimulator()
  : Node("nusimulator")
  {
    // Initialize the timer Parameter:
    auto rate = declare_parameter<double>("rate", 100.0);
    // Initialize the ~/timestep Publisher:
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    // Initialize the ~/real_walls and ~/real_obstacles Publishers:
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
      rclcpp::QoS(10).transient_local());
    obst_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles",
      rclcpp::QoS(10).transient_local());
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Initialize the Publisher for Sensor Data:
    sensor_data_publisher_ =
      this->create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    // Initialize the Publisher for Joint States:
    joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

    // Initialize and publish the Real Walls:
    auto marker_array_walls = createWalls();
    marker_pub_->publish(marker_array_walls);

    // Initialize and publish the Real Obstacles:
    auto marker_array_obstacles = createObstacles();
    obst_pub_->publish(marker_array_obstacles);

    // Create the timer callback:
    auto timer_callback = [this, rate]() -> void {
        // Print a Message Once:
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Timer rate is " << rate << " Hz!");

        // Timerate Publisher:
        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;
        RCLCPP_DEBUG_STREAM_ONCE(this->get_logger(), "Timestep:" << message.data);
        this->publisher_->publish(message);

        // Update the wheel positions and publish them on red/sensor_data topic:
        auto dt = 1.0 / rate; // Change in time since last publish.

        // Update the DiffDrive model using the current wheel positions:
        double phi_left = turtlelib::normalize_angle(diff_drive_.get_phi_left() + wheel_vel_.v_lw *
        dt);
        double phi_right = turtlelib::normalize_angle(diff_drive_.get_phi_right() +
        wheel_vel_.v_rw * dt);
        diff_drive_.update_fk(phi_left, phi_right);

        // Publish the SensorData message update:
        nuturtlebot_msgs::msg::SensorData sensor_data_msg;
        sensor_data_msg.stamp = this->get_clock()->now();
        sensor_data_msg.left_encoder = static_cast<int>(diff_drive_.get_phi_left() *
          encode_ticks_per_rad_);
        sensor_data_msg.right_encoder = static_cast<int>(diff_drive_.get_phi_right() *
          encode_ticks_per_rad_);
        sensor_data_publisher_->publish(sensor_data_msg);

        // Publish the JointState message update:
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = sensor_data_msg.stamp;
        // Set joint positions:
        joint_state_msg.name.push_back("wheel_left_joint");
        joint_state_msg.position.push_back(phi_left);
        joint_state_msg.name.push_back("wheel_right_joint");
        joint_state_msg.position.push_back(phi_right);
        joint_state_publisher_->publish(joint_state_msg);

        RCLCPP_DEBUG_STREAM(this->get_logger(),
        "The phi_left is " << phi_left << " and phi_right is " << phi_right << " at rate " <<
          rate << " Hz!");

        // Update the robot positions based on the internal diffdrive:
        x0_ = diff_drive_.get_q().translation().x;
        y0_ = diff_drive_.get_q().translation().y;
        theta0_ = diff_drive_.get_q().rotation();

        // Assign parameters to corresponding tf variables and broadcast:
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = sensor_data_msg.stamp;
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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate)),
      timer_callback);
    // Reset service
    reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&Nusimulator::handle_service_reset, this, std::placeholders::_1,
      std::placeholders::_2)
    );

    // Construct the subscriber for Wheel Commands to the red Ground Robot:
    wheel_cmd_subscriber_ =
      this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd", 10,
        [this](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
        // Log the received wheel commands:
          RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
        "Received wheel commands - Left: " << msg->left_velocity << ", Right: " <<
            msg->right_velocity);
        // Original Thought - Remove if other idea works fine:
          wheel_vel_.v_lw = std::clamp(static_cast<double>(msg->left_velocity) /
        this->motor_cmd_per_rad_sec_, -(this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_),
          (this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_));
          wheel_vel_.v_rw = std::clamp(static_cast<double>(msg->right_velocity) /
        this->motor_cmd_per_rad_sec_, -(this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_),
          (this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_));
    });
  }

private:
/// \brief Creates visualization markers for the arena walls and floor
///
/// \returns A MarkerArray containing 4 walls (red) and a floor (white) with dimensions
///          set by the parameters arena_x_length, arena_y_length, and arena_thickness
  visualization_msgs::msg::MarkerArray createWalls()
  {
    // Initialize and publish the Real Walls:
    visualization_msgs::msg::MarkerArray marker_array_walls;
    for (int i = 0; i < 5; ++i) {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "nusim/world";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "red";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // Set orientation
      marker.pose.orientation.w = 1.0;

      // Determine wall locations
      if (i == 0) {
        // Bottom Wall
        marker.pose.position.x = 0.0;
        marker.pose.position.y = -arena_y_ / 2.0 - arena_thick_ / 2.0;
        marker.pose.position.z = arena_thick_ / 2.0;
        marker.scale.x = arena_x_ + arena_thick_ * 2.0;
        marker.scale.y = arena_thick_;
        marker.scale.z = arena_thick_;
      } else if (i == 1) {
        // Left Wall
        marker.pose.position.x = -arena_x_ / 2.0 - arena_thick_ / 2.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = arena_thick_ / 2.0;
        marker.scale.x = arena_thick_;
        marker.scale.y = arena_y_ + arena_thick_ * 2.0;
        marker.scale.z = arena_thick_;
      } else if (i == 2) {
        // Right Wall
        marker.pose.position.x = arena_x_ / 2.0 + arena_thick_ / 2.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = arena_thick_ / 2.0;
        marker.scale.x = arena_thick_;
        marker.scale.y = arena_y_ + arena_thick_ * 2.0;
        marker.scale.z = arena_thick_;
      } else if (i == 3) {
        // Top Wall
        marker.pose.position.x = 0.0;
        marker.pose.position.y = arena_y_ / 2.0 + arena_thick_ / 2.0;
        marker.pose.position.z = arena_thick_ / 2.0;
        marker.scale.x = arena_x_ + arena_thick_ * 2.0;
        marker.scale.y = arena_thick_;
        marker.scale.z = arena_thick_;
      } else if (i == 4) {
        // Floor
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = -arena_thick_ / 2.0;
        marker.scale.x = arena_x_;
        marker.scale.y = arena_y_;
        marker.scale.z = arena_thick_;
      }
      // Colors
      if (i < 4) {
        // Arena walls (red)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      } else if (i == 4) {
        // Floor (white)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
      }
      marker_array_walls.markers.emplace_back(marker);
    }
    return marker_array_walls;
  }
  /// \brief Creates visualization markers for obstacles
  ///
  /// \returns A MarkerArray containing cylinders representing obstacles at positions
  ///          obstacles_x, obstacles_y with radii obstacles_r and height obstacle_height
  visualization_msgs::msg::MarkerArray createObstacles()
  {
    // Initialize and publish the Real Obstacles:
    visualization_msgs::msg::MarkerArray marker_array_obstacles;

    // Check that sizes match
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR(this->get_logger(), "obstacles_x and obstacles_y are different lengths!!!");
      throw std::runtime_error("Obstacle coordinate size mismatch in lengths...");
    }

    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "nusim/world";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "red";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // Set orientation
      marker.pose.orientation.w = 1.0;
      // Determine obstacle locations and size:
      marker.pose.position.x = obstacles_x_.at(i);
      marker.pose.position.y = obstacles_y_.at(i);
      marker.pose.position.z = obst_height_ / 2;
      marker.scale.x = obstacles_r_.at(i) * 2;
      marker.scale.y = obstacles_r_.at(i) * 2;
      marker.scale.z = obst_height_;
      // Color Obstacles (Red)
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      // Add obstacle to marker array
      marker_array_obstacles.markers.emplace_back(marker);
    }
    return marker_array_obstacles;
  }
  /// \brief Resets the simulation timestep and robot initial pose
  ///
  /// Resets timestep_ to 0 and sets x0_, y0_, theta0_ to their declared parameters.
  ///
  /// \param request - Empty request from the std_srvs::srv::Empty service
  /// \param response - Empty response from the std_srvs::srv::Empty service
  /// \returns void
  void handle_service_reset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
    RCLCPP_INFO(this->get_logger(), "Timestep Reset!");
    timestep_ = 0;                  // Reset the timestep counter
    // ####################### Begin_Citation [7] ############################
    // Reset the simulation positions:
    x0_ = this->get_parameter("x0").as_double();
    y0_ = this->get_parameter("y0").as_double();
    theta0_ = this->get_parameter("theta0").as_double();
    // ######################## End_Citation [7] ###############################
  }

  // Initialize ROS 2 Infrustructure:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obst_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Set the timestep:
  uint64_t timestep_;

  // Set the Ground Truth values:
  double x0_ = this->declare_parameter<double>("robot.x0", 0.0);
  double y0_ = this->declare_parameter<double>("robot.y0", 0.0);
  double theta0_ = this->declare_parameter<double>("robot.theta0", 0.0);

  // Set the Arena Wall Dimensions:
  double arena_x_ = this->declare_parameter<double>("arena_x_length", 8.0);
  double arena_y_ = this->declare_parameter<double>("arena_y_length", 12.0);
  double arena_thick_ = this->declare_parameter<double>("arena_thickness", 0.25);

  // Set the Obstacle Dimensions:
  std::vector<double> obstacles_x_ = this->declare_parameter<std::vector<double>>("obstacles_x",
    std::vector<double>{});
  std::vector<double> obstacles_y_ = this->declare_parameter<std::vector<double>>("obstacles_y",
    std::vector<double>{});
  std::vector<double> obstacles_r_ = this->declare_parameter<std::vector<double>>("obstacles_r",
    std::vector<double>{});
  double obst_height_ = this->declare_parameter<double>("obstacle_height", 0.25);

  // Initialize the parameters:
  double wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
  double track_width_ = declare_parameter<double>("track_width", 0.16);
  int motor_cmd_max_ = declare_parameter<int>("motor_cmd_max", 265);
  double motor_cmd_per_rad_sec_ = declare_parameter<double>("motor_cmd_per_rad_sec", 41.67);
  int encode_ticks_per_rad_ = declare_parameter<int>("encode_ticks_per_rad", 652);

  // Initialize the DiffDrive model:
  turtlelib::DiffDrive diff_drive_{track_width_, wheel_radius_};

  // Store Wheel velocities:
  turtlelib::wheel_vel wheel_vel_{0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}
