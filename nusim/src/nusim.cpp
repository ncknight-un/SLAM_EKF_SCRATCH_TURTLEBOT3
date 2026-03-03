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
///     ~/fake_obstacles (visualization_msgs::msg::MarkerArray): Markers representing fake obstacles for sensor data
///     ~/sim_scan (sensor_msgs::msg::LaserScan): Simulated laser scan data based on the robot's position and obstacles in the arena
///     ~/red/nav_path (nav_msgs::msg::Path): The current traced path of the robot.
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
#include <random>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
  static std::random_device rd{};
  static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
  return mt;
}

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
    fake_obst_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_obstacles",
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

    // Initialize the Publisher for the robot path:
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("red/nav_path", 10);

    // Initialize the Publisher for the fake laser scan data:
    fake_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("red/sim_scan", 10);

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

        // Apply slip and noise to the ground truth:
        // Update the DiffDrive model using the current wheel positions:
        std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);

        double slip_l = slip_dist(get_random());
        double slip_r = slip_dist(get_random());

        // Update just the encoder phi_left and right for sensor data:
        double phi_left = turtlelib::normalize_angle(diff_drive_no_noise_.get_phi_left() +
        wheel_vel_.v_lw * dt);
        double phi_right = turtlelib::normalize_angle(diff_drive_no_noise_.get_phi_right() +
        wheel_vel_.v_rw * dt);

        // Slip only the increment, not the total accumulated angle:
        double phi_left_noise = turtlelib::normalize_angle(diff_drive_.get_phi_left() +
        wheel_vel_.v_lw * dt * (1.0 + slip_l));
        double phi_right_noise = turtlelib::normalize_angle(diff_drive_.get_phi_right() +
        wheel_vel_.v_rw * dt * (1.0 + slip_r));

        diff_drive_.update_fk(phi_left_noise, phi_right_noise);  // ground truth with slip
        diff_drive_no_noise_.update_fk(phi_left, phi_right); // odometry without slip, but with noise in the sensor data

        RCLCPP_DEBUG_STREAM(this->get_logger(),
        "The phi_left is " << phi_left << " and phi_right is " << phi_right << " at rate " <<
          rate << " Hz!");

        // Publish the SensorData message update:
        nuturtlebot_msgs::msg::SensorData sensor_data_msg;
        sensor_data_msg.stamp = this->get_clock()->now();
        sensor_data_msg.left_encoder = static_cast<int>(diff_drive_no_noise_.get_phi_left() *
          encode_ticks_per_rad_);
        sensor_data_msg.right_encoder = static_cast<int>(diff_drive_no_noise_.get_phi_right() *
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

        // Check for collision and update the robot position if a collision is detected:
        auto [collision_detected, obstacle_index] = checkCollision();
        if (collision_detected) {
          updateCollision(obstacle_index);
          // ############################### Begin_Citation [13] ##################################
          // Update the DiffDrive model with the new position after collision:
          diff_drive_.set_q(turtlelib::Transform2D(turtlelib::Vector2D(x0_, y0_),
          turtlelib::normalize_angle(theta0_)));
          // ############################### End_Citation [13] ##################################
        }

        // Check for wall collision and update the robot position if a collision is detected:
        auto [wall_collision_detected, wall_index] = checkWallCollision();
        if(wall_collision_detected) {
          updateWallCollision(wall_index);
          diff_drive_.set_q(turtlelib::Transform2D(turtlelib::Vector2D(x0_, y0_),
          turtlelib::normalize_angle(theta0_)));
        }

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

        // Publish the robot path:
        robot_path_.header.stamp = sensor_data_msg.stamp;  // Keep the same timestamp as sensor data update
        robot_path_.header.frame_id = "nusim/world";
        geometry_msgs::msg::PoseStamped pose;
        pose.header = robot_path_.header;
        // Update the pose of the robot:
        pose.pose.position.x = x0_;
        pose.pose.position.y = y0_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        robot_path_.poses.push_back(pose);
        path_publisher_->publish(robot_path_);
      };

    auto fake_sensor_callback_ = [this]() -> void {
      // Create a timer callback for publishing the fake sensor data of the obstacles every 200 ms:
        auto marker_array_fake_obstacles = createFakeObstacles();
        fake_obst_pub_->publish(marker_array_fake_obstacles);

      // ###################################### Begin_Citation [12] ######################################
      // Update the fake lidar scan data and publish it on red/sim_scan topic:
        sensor_msgs::msg::LaserScan laser_scan_msg;
        laser_scan_msg.header.stamp = this->get_clock()->now();
        laser_scan_msg.header.frame_id = "red/base_footprint"; // Turtlebot3 laser scan frame is shifted, so I used a centered one.

      // Set the laser scan parameters:
        laser_scan_msg.angle_min = laser_angle_min_;
        laser_scan_msg.angle_max = laser_angle_max_;
        laser_scan_msg.angle_increment = laser_angle_increment_;
        laser_scan_msg.time_increment = (1.0 / 200.0) / laser_num_readings_; // Time between individual measurements, assuming all measurements are taken in 1/200 seconds as publlished by the timer./
        laser_scan_msg.scan_time = 0.2; // Scan data is published every 200 ms.
        laser_scan_msg.range_min = laser_min_range_;
        laser_scan_msg.range_max = laser_max_range_;

      // Initialize the ranges vector:
        laser_scan_msg.ranges.resize(laser_num_readings_, laser_max_range_);

      // Loop through the scanner and see if it collides with an obstacle or wall:
        for(int i = 0; i < laser_num_readings_; i++) {
        // Calculate the angle of the current laser scan ray in robot frame:
          double angle = laser_scan_msg.angle_min + i * laser_scan_msg.angle_increment;

        // Calculate the end point of the laser ray in the robot frame:
          double ray_x = laser_scan_msg.range_max * std::cos(angle + theta0_);
          double ray_y = laser_scan_msg.range_max * std::sin(angle + theta0_);
          turtlelib::Vector2D ray_dir(ray_x, ray_y);

        // Normalize the ray direction vector:
          turtlelib::Vector2D ray_dir_normalized = turtlelib::normalize(ray_dir);
          auto ray_dist = turtlelib::magnitude(ray_dir);

        // Check for intersection with each obstacle:
          for(size_t obs = 0; obs < obstacles_x_.size(); obs++) {
            auto obs_x = obstacles_x_.at(obs);
            auto obs_y = obstacles_y_.at(obs);
            auto obs_r = obstacles_r_.at(obs);

          // Get the vector of the turtlebot to the obstacle center:
            turtlelib::Vector2D to_obstacle(obs_x - x0_, obs_y - y0_);

          // Normalize the ray vector to get the direction, then multiply by the object distance:
            auto obstacle_distance = turtlelib::magnitude(to_obstacle);

          // Project the center onto the ray to get the closest point on the ray to the center of the obstacle:
            auto projection_length = turtlelib::dot(to_obstacle, ray_dir_normalized);

          // Compute perpendicular distance from the obstacle center to the ray:
            auto perpendicular_dist = std::sqrt(std::pow(obstacle_distance,
            2) - std::pow(projection_length, 2));

          // Check if the distance between the two vectors is less than the obstacle radius:
            if(perpendicular_dist <= obs_r && projection_length > 0 &&
              projection_length < ray_dist)
            {
            // Distance from projection point to the intersection point on the ray:
              auto intersection_dist = std::sqrt(std::pow(obs_r, 2) - std::pow(perpendicular_dist,
              2));

            // Find the first intersection point along the ray:
              auto laser_hit_point = ray_dir_normalized * (projection_length - intersection_dist);

            // If the ray intersects with the obstacle, calculate the distance from the robot to the point of intersection:
              double distance_to_obstacle = turtlelib::magnitude(laser_hit_point);
            // Check if this distance is less than the current range reading for this ray and within the laser scan range limits:
              if (distance_to_obstacle < laser_scan_msg.ranges[i] &&
                distance_to_obstacle<laser_scan_msg.range_max &&
                distance_to_obstacle> laser_scan_msg.range_min)
              {
              // Update the range reading for this ray to be the distance to the obstacle plus some gaussian noise based on the laser scan variance parameter:
                std::normal_distribution<> d(0.0, std::sqrt(laser_scan_variance_));
                laser_scan_msg.ranges[i] = distance_to_obstacle + d(get_random());
              }
            }
          }
        // Check for wall Intersections using line-line intersection between the ray and each wall segment:
        // Loop through each wall segment and check for intersection with the laser:
          for(size_t w = 0; w < 4; ++w) {
            double x1, y1, x2, y2;
            switch(w) {
              case 0: // Bottom wall
                x1 = -arena_x_ / 2.0;
                x2 = arena_x_ / 2.0;
                y2 = y1 = -arena_y_ / 2.0 + arena_thick_ / 2.0;
                break;
              case 1: // Left wall
                x2 = x1 = -arena_x_ / 2.0 + arena_thick_ / 2.0;
                y1 = -arena_y_ / 2.0;
                y2 = arena_y_ / 2.0;
                break;
              case 2: // Right wall
                x1 = x2 = arena_x_ / 2.0 - arena_thick_ / 2.0;
                y1 = -arena_y_ / 2.0;
                y2 = arena_y_ / 2.0;
                break;
              case 3: // Top wall
                x1 = -arena_x_ / 2.0;
                x2 = arena_x_ / 2.0;
                y2 = y1 = arena_y_ / 2.0 - arena_thick_ / 2.0;
                break;
              default: continue;
            }
          // ################################ Begin_Citation [14] ######################################
          // Check for intersection between the ray and the wall segment:
            double ray_dx = std::cos(angle + theta0_);
            double ray_dy = std::sin(angle + theta0_);

            double seg_dx = x2 - x1;
            double seg_dy = y2 - y1;
            double denom = ray_dx * seg_dy - ray_dy * seg_dx;
          // Check to see if laser is parallel to wall segment:
            if(std::abs(denom) < 1e-9) {continue;}

          // laser is not parallel, so calculate line intersection parameters t and u:
            double t = ((x1 - x0_) * seg_dy - (y1 - y0_) * seg_dx) / denom;
            double u = ((x1 - x0_) * ray_dy - (y1 - y0_) * ray_dx) / denom;
          // ################################## End_Citation [14] ######################################

            if(t > laser_scan_msg.range_min && t < laser_scan_msg.range_max && u >= 0.0 &&
              u <= 1.0)
            {
              if(t < laser_scan_msg.ranges[i]) {
                std::normal_distribution<> d(0.0, std::sqrt(laser_scan_variance_));
                laser_scan_msg.ranges[i] = t + d(get_random());
              }
            }
          }
        }
      // ####################################### End_Citation [12] ######################################
      // Set the intensities to empty:
        laser_scan_msg.intensities = std::vector<float>(laser_scan_msg.ranges.size(), 0.0); // Fill with all zeros since we don't use intensity. When I left empty it would reset my Rviz each build.
      // Publish the laser scan message:
        fake_laser_pub_->publish(laser_scan_msg);
      };

    // Set the timer:
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate)),
      timer_callback);

    // Create a timer to publish the fake sensor data every 200 ms:
    sensor_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), fake_sensor_callback_);

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

      // Apply Input noise to velocity control:
          auto ur = msg->right_velocity;
          auto ul = msg->left_velocity;

      // Set the normal distribuition for noise:
          std::normal_distribution<> d(0.0, input_noise_);
          auto vr = ur + d(get_random());
          auto vl = ul + d(get_random());
      // Update vr and vl if ur or ul is 0.0:
          if(ur == 0) {
            vr = ur;
          }
          if(ul == 0) {
            vl = ul;
          }

      // Set wheel velocities with update to vi:
          wheel_vel_.v_lw = std::clamp(static_cast<double>(vl) / this->motor_cmd_per_rad_sec_,
        -(this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_),
          (this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_));
          wheel_vel_.v_rw = std::clamp(static_cast<double>(vr) / this->motor_cmd_per_rad_sec_,
        -(this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_),
          (this->motor_cmd_max_ / this->motor_cmd_per_rad_sec_));
    });
  }

private:
/// \brief A function to check collision detection between the robot and the obstacles in the arena. This will be called after every position update.
///
/// \returns A boolean value indicating if the collision radius of the robot overlaps with the collision radius of an obstacle, and the index of the object collided with.
  std::pair<bool, int> checkCollision()
  {
    // Check for collision with obstacles:
    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      double obs_x = obstacles_x_.at(i);
      double obs_y = obstacles_y_.at(i);
      double obs_r = obstacles_r_.at(i);
      // Calculate the distance between the robot and the obstacle centers:
      double distance = std::sqrt(std::pow(x0_ - obs_x, 2) + std::pow(y0_ - obs_y, 2));
      if (distance < (collision_radius_ + obs_r)) {
        RCLCPP_WARN_STREAM(this->get_logger(),
          "Collision detected with obstacle at (" << obs_x << ", " << obs_y << ")!");
        return std::make_pair(true, i); // Collision detected with an obstacle at index i
      }
    }
    return std::make_pair(false, -1); // No collision after update, return -1 for index to indicate no collision.
  }

/// \brief Update the center of the robot after a collision is detected. This will be called after checkCollision() returns true.
///
/// \returns void, updates the posiiton of the robots center.
  void updateCollision(int obstacle_index)
  {
    // Get the position of the obstacle the robot collided with:
    double obs_x = obstacles_x_.at(obstacle_index);
    double obs_y = obstacles_y_.at(obstacle_index);
    double obs_r = obstacles_r_.at(obstacle_index);

    // Calculate the line angle between the two centers of the robot and the obstacle (Heading from the obstacle to the robot):
    double angle_to_robot = std::atan2(y0_ - obs_y, x0_ - obs_x);
    // Calculate the new position of the robot center by shifting it back along the angle until the collision radius of the robot is tangent to the obstacle:
    // (collision_radius_ + obs_r) is the distance from the obstacle center to the point of tangency, so we shift along that heading.
    double new_x = obs_x + (collision_radius_ + obs_r) * std::cos(angle_to_robot);
    double new_y = obs_y + (collision_radius_ + obs_r) * std::sin(angle_to_robot);
    // Update the robot center position:
    x0_ = new_x;
    y0_ = new_y;
    return;
  }
/// \brief A function to check collision determination between the robot and the arena walls. This will be called after every position update.
///
/// \returns A boolean value indicating if the collision radius of the robot overlaps with the arena walls, and the index of the wall collided with.
  std::pair<bool, int> checkWallCollision()
  {
    // Check for collision with walls one by one:
    // Check bottom wall:
    if ((y0_) < (-arena_y_ / 2.0 + arena_thick_ / 2.0)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Collision detected with bottom wall!");
      return std::make_pair(true, 0); // Collision detected with bottom wall
    }
    // Check left wall:
    if ((x0_) < (-arena_x_ / 2.0 + arena_thick_ / 2.0)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Collision detected with left wall!");
      return std::make_pair(true, 1); // Collision detected with left wall
    }
    // Check right wall:
    if ((x0_) > (arena_x_ / 2.0 - arena_thick_ / 2.0)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Collision detected with right wall!");
      return std::make_pair(true, 2); // Collision detected with right wall
    }
    // Check top wall:
    if ((y0_) > (arena_y_ / 2.0 - arena_thick_ / 2.0)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Collision detected with top wall!");
      return std::make_pair(true, 3); // Collision detected with top wall
    }
    // No collision is detected with walls, return false and index -1.
    return std::make_pair(false, -1);
  }

/// \brief Update the center of the robot after a collision with the wall is detected. This will be called after checkWallCollision() returns true.
///
/// \returns void, but will update robots position to be tangent to the wall.
  void updateWallCollision(int wall_index)
  {
    // Update the robot center position based on which wall was collided with:
    switch (wall_index) {
      case 0: // Bottom wall
        y0_ = -arena_y_ / 2.0 + arena_thick_ / 2.0;
        break;
      case 1: // Left wall
        x0_ = -arena_x_ / 2.0 + arena_thick_ / 2.0;
        break;
      case 2: // Right wall
        x0_ = arena_x_ / 2.0 - arena_thick_ / 2.0;
        break;
      case 3: // Top wall
        y0_ = arena_y_ / 2.0 - arena_thick_ / 2.0;
        break;
      default:
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid wall index in updateWallCollision!");
    }
    return;
  }

/// \brief Creates visualization markers for the fake sensor data of the obstacles.
///
/// \returns A MarkerArray containing a gaussian distribution of fake obstacles that dissapear/are
///           deleted when they go out of the robots max range.
  visualization_msgs::msg::MarkerArray createFakeObstacles()
  {
    // Initialize and publish the Fake Obstacles:
    visualization_msgs::msg::MarkerArray marker_array_fake_obstacles;

    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      visualization_msgs::msg::Marker marker;

      // Publish the markers with respect to the world frame, but check against the robots positon.
      // ############################## Begin_Citation [13] ##################################
      marker.header.frame_id = "red/base_footprint";
      // ############################### End_Citation [13] ###################################
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "fake_obstacles";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // Set orientation
      marker.pose.orientation.w = 1.0;

      // Determine obstacle locations and size:
      // Add uncertainty radius to obstacles:
      auto sigma = std::sqrt(basic_sensor_variance_);
      std::normal_distribution<> d(0.0, sigma);
      // World-frame obstacle position (with noise)
      double obs_x_world = obstacles_x_.at(i) + d(get_random());
      double obs_y_world = obstacles_y_.at(i) + d(get_random());
      // Transform into robot frame to publish as a fake sensor data:
      double dx = obs_x_world - x0_;
      double dy = obs_y_world - y0_;
      double obs_x_robot = std::cos(theta0_) * dx + std::sin(theta0_) * dy;
      double obs_y_robot = -std::sin(theta0_) * dx + std::cos(theta0_) * dy;
      double obs_r = obstacles_r_.at(i) + d(get_random());

      // Set the marker information for the fake obstacles:
      marker.pose.position.x = obs_x_robot;
      marker.pose.position.y = obs_y_robot;
      marker.pose.position.z = obst_height_ / 2;
      marker.scale.x = obs_r * 2;
      marker.scale.y = obs_r * 2;
      marker.scale.z = obst_height_;
      // Fake Obstacles (Yellow)
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      // Add obstacle to marker array
      marker_array_fake_obstacles.markers.emplace_back(marker);

      // If the obstacle is out of range of the robot in the world, delete it from the marker array by changing its action to DELETE:
      double distance = std::sqrt(std::pow(obs_x_robot - 0.0, 2) + std::pow(obs_y_robot - 0.0, 2));
      if (distance > max_range_) {
        marker_array_fake_obstacles.markers.back().action = visualization_msgs::msg::Marker::DELETE;
      }
    }
    return marker_array_fake_obstacles;
  }

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
        marker.pose.position.z = -arena_thick_ / 2.0 - 0.05; // Slightly below the walls to avoid z-fighting in visualization
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
        // Floor (Gray)
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
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
      // Color Obstacles (Red)          diff_drive_.set_q(turtlelib::Transform2D(turtlelib::Vector2D(x0_, y0_), turtlelib::normalize_angle(theta0_)));

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
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obst_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obst_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_laser_pub_;

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
  double input_noise_ = declare_parameter<double>("input_noise", 0);
  double slip_fraction_ = declare_parameter<double>("slip_fraction", 0);
  double max_range_ = declare_parameter<double>("max_range", 2.0);
  double basic_sensor_variance_ = declare_parameter<double>("basic_sensor_variance", 0.01);
  double collision_radius_ = declare_parameter<double>("collision_radius", 0.11);

  // Set the laser parameters:
  double laser_max_range_ = declare_parameter<double>("laser_max_range", 3.5);
  double laser_min_range_ = declare_parameter<double>("laser_min_range", 0.12);
  double laser_angle_min_ = declare_parameter<double>("laser_angle_min", -3.14);
  double laser_angle_max_ = declare_parameter<double>("laser_angle_max", 3.14);
  double laser_angle_increment_ = declare_parameter<double>("laser_angle_increment", 0.01745);
  int laser_num_readings_ = declare_parameter<int>("laser_num_readings", 360);
  double laser_resolution_ = declare_parameter<double>("laser_resolution", 0.01745);
  double laser_scan_variance_ = declare_parameter<double>("laser_scan_variance", 0.01);

  // Initialize the DiffDrive model for the ground truth (with noise):
  turtlelib::DiffDrive diff_drive_{track_width_, wheel_radius_};
  // Initialize the DiffFrive model for the odometry publishing (without noise):
  turtlelib::DiffDrive diff_drive_no_noise_{track_width_, wheel_radius_};

  // Store Wheel velocities:
  turtlelib::wheel_vel wheel_vel_{0.0, 0.0};

  // Store the robot path:
  nav_msgs::msg::Path robot_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusimulator>());
  rclcpp::shutdown();
  return 0;
}
