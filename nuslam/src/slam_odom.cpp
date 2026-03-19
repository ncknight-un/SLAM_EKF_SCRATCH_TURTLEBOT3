/// \file
/// \brief Creates a ROS 2 Node which publishes odimetry messages and the odometry transform.
///
/// PARAMETERS:
///     body_id (string): The name of the robot's body frame to use in the odometry transform.
///     odom_id (string): The name of the odometry frame to use in the odometry transform.
///     wheel_left (string): The name of the left wheel joint to read from the joint_states topic.
///     wheel_right (string): The name of the right wheel joint to read from the joint_states topic.
///     wheel_radius (double): The radius of the robot's wheels in meters, used to compute odometry.
///     track_width (double): The distance between the robot's wheels in meters, used to compute odometry.
///     slam_process_variance (double): The process variance to use in the EKF SLAM algorithm.
///     slam_measurement_variance (double): The measurement variance to use in the EKF SLAM algorithm.
/// PUBLISHES:
///     ~ odom (nav_msgs/msg/Odometry): The current estimated pose and velocity of the robot base in the odom frame.
///     ~ slam_obstacles (visualization_msgs::msg::MarkerArray): The current set of fake obstacles in the SLAM map.
///     ~ slam_path (nav_msgs/msg/Path): The current path of the robot based on its position updates for EKF SLAM.
/// SUBSCRIBES:
///     ~ joint_states (sensor_msgs/msg/JointState): The current joint states of the odom robot, which is used to update the internal odometry state and publish the current odometry message and transform.
///     ~ fake_obstacles (visualization_msgs::msg::MarkerArray): Markers representing fake obstacles for sensor data with Gaussian Noise (mimics sensor data)
/// SERVERS:
///     ~ initial pose - Resets the robots odometry to think it is at the requested configuration.
/// CLIENTS:
///     None

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "slamlib/ekf.hpp"
#include <nav_msgs/msg/path.hpp>
#include <armadillo>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/// \brief A class to launch a Simulator Node
class SlamOdometry : public rclcpp::Node {
public:
  SlamOdometry()
  : Node("slam_odom")
  {
        // Check required parameters:
    if (wheel_left_.empty()) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
        "Parameter 'wheel_left' is required but not set. Exiting...");
      rclcpp::shutdown();        // terminate the node
      return;
    }
    if (wheel_right_.empty()) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
        "Parameter 'wheel_right' is required but not set. Exiting...");
      rclcpp::shutdown();       // terminate the node
      return;
    }

        // Initialize the transform broadcaster
    tf_broadcaster_odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

       // Initialize the SLAM transform broadcaster
    tf_broadcaster_slam_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Construct the publisher for odometry:
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Construct the publisher for SLAM path:
    slam_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("green/slam_path", 10);

        // Construct the publisher for SLAM obstacles:
    slam_obstacle_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("slam_obstacles", 10);

        // Construct the subscriber for Fake Obstacle Sensor Data with Noise:
    fake_obstacles_subscriber_ =
      this->create_subscription<visualization_msgs::msg::MarkerArray>("fake_obstacles", 100,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // Break apart the Marker array, and update the SLAM EKF with each obstacle measurement:
          for(const auto & marker : msg->markers) {
            // Pull out each marker and posiiton that is in the robot frame (sensor measurements)
            auto obs_x = marker.pose.position.x;
            auto obs_y = marker.pose.position.y;

            // Range and Bearing:
            auto range = std::sqrt(obs_x * obs_x + obs_y * obs_y);
            auto bearing = turtlelib::normalize_angle(std::atan2(obs_y, obs_x));
            // Build colvec measurement for EKF update:
            arma::colvec measurement(2);
            measurement(0) = range;
            measurement(1) = bearing;

            // Get the obstacle id from No Data Association Algorithm:
            auto obs_id = slam_ekf_.dataAssociation(measurement, mahalanobis_threshold);

            if(obs_id == -1) {
              RCLCPP_DEBUG_STREAM(this->get_logger(),
                "New obstacle detected at range=" << range << ", bearing=" << bearing);
              // Since this is a new landmark, add it to the EKF SLAM state:
              slam_ekf_.addLandmark(measurement);
              obs_id = slam_ekf_.getNumLandmarks() - 1; // Get the index of the new landmark that was just added.
            } else {
              RCLCPP_DEBUG_STREAM(this->get_logger(),
                "Existing obstacle " << obs_id << " observed again at range=" << range
                                     << ", bearing=" << bearing);
            }

            // Update the SLAM EKF with the new obstacle data:
            slam_ekf_.updateEKF(measurement, obs_id);
          }

        // Get the new SLAM EKF state and obstacles/landmarks and publish it as a MarkerArray for visualization:
          auto slam_obstacles = slam_ekf_.getLandmarkPositions(); // vector of points.
          auto slam_pose = slam_ekf_.getState(); // Transform2D of the robot in the SLAM map.

        // Convert the SLAM EKF landmark positions to a MarkerArray:
          visualization_msgs::msg::MarkerArray marker_array_slam_obstacles =
          createSLAMObstacles(slam_obstacles);
          slam_obstacle_pub_->publish(marker_array_slam_obstacles);

        // Compute map->odom:
        // slam_pose times the inv(odom_pose)
          turtlelib::Transform2D map_to_odom = slam_pose * diff_drive_.get_q().inv();

        // Publish the SLAM EKF robot pose as a transform:
          geometry_msgs::msg::TransformStamped t;

          // Make sure that there is at least one marker seen before updating tf:
          if(msg->markers.empty()) {
            return;
          }

          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "nusim/world"; // Publish the SLAM EKF pose in the world frame (remapped map frame)
          t.child_frame_id = odom_id_;
          t.transform.translation.x = map_to_odom.translation().x;
          t.transform.translation.y = map_to_odom.translation().y;
          t.transform.translation.z = 0.0;
          tf2::Quaternion q;
          q.setRPY(0, 0, map_to_odom.rotation());
          t.transform.rotation.x = q.x();
          t.transform.rotation.y = q.y();
          t.transform.rotation.z = q.z();
          t.transform.rotation.w = q.w();
          tf_broadcaster_slam_->sendTransform(t);

        // Publish the SLAM obstacles based on the MAP update from the EKF SLAM algorithm:
          slam_path_.header.stamp = t.header.stamp;
          slam_path_.header.frame_id = "nusim/world";
          geometry_msgs::msg::PoseStamped pose;
          pose.header = slam_path_.header;
        // Update the pose of the robot:
          pose.pose.position.x = slam_pose.translation().x;
          pose.pose.position.y = slam_pose.translation().y;
          pose.pose.position.z = 0.0;
        // Orienation of the robot as a quaternion:
          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();
          slam_path_.poses.push_back(pose);
          slam_path_pub_->publish(slam_path_);

        });

        // Construct the subscriber for joint states:
    joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Update the internal odemetry state every time a new joint state message is received, and publish the current odometry message and transform:
            // Find indices of wheel joints in joint state message and set a map:
          static std::unordered_map<std::string, size_t> joint_map;
          if (joint_map.empty()) {
            for (size_t i = 0; i < msg->name.size(); i++) {
                    // Check to see if joint name matches:
              joint_map[msg->name[i]] = i;
            }
          }
          double phi_left = msg->position[joint_map[wheel_left_]];
          double phi_right = msg->position[joint_map[wheel_right_]];

            // Compute Twist
          turtlelib::Twist2D tw = diff_drive_.compute_twist(turtlelib::Vector2D{phi_left,
            phi_right});

            //  ################# SLAM EKF PREDICTION STEP #################
          if(tw.x != 0.0 || tw.omega != 0.0) {
                // Movement occurs, update prediction:
            slam_ekf_.predict(tw);
          }

          RCLCPP_DEBUG_STREAM(this->get_logger(), "tw.x=" << tw.x << " tw.omega=" << tw.omega);
            // #############################################################

            // Update forward kinematics:
          diff_drive_.update_fk(phi_left, phi_right);

            // ------------------------------------------------------------------------------
            // Build and publish Odometry and TF:
          nav_msgs::msg::Odometry odometry_msg;
          odometry_msg.header.stamp = msg->header.stamp;
          odometry_msg.header.frame_id = odom_id_;
          odometry_msg.child_frame_id = body_id_;

            // Set pose in body frame:
          odometry_msg.pose.pose.position.x = diff_drive_.get_q().translation().x;
          odometry_msg.pose.pose.position.y = diff_drive_.get_q().translation().y;
          odometry_msg.pose.pose.position.z = 0.0;
          tf2::Quaternion q;
          q.setRPY(0, 0, diff_drive_.get_q().rotation());
          odometry_msg.pose.pose.orientation.x = q.x();
          odometry_msg.pose.pose.orientation.y = q.y();
          odometry_msg.pose.pose.orientation.z = q.z();
          odometry_msg.pose.pose.orientation.w = q.w();
          odometry_msg.pose.covariance = {0.0};   // Set covariance to 0.

            // Set the Linear and angular velocity relative to body frame:
            // Get the current twist from the DiffDrive model:
          odometry_msg.twist.twist.linear.x = tw.x;
          odometry_msg.twist.twist.linear.y = tw.y;
          odometry_msg.twist.twist.angular.z = tw.omega;
          odometry_msg.twist.covariance = {0.0};   // Set covariance to 0.
            // Publish the odometry message:
          odometry_publisher_->publish(odometry_msg);

            // Publish the odometry transform:
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = msg->header.stamp;
          t.header.frame_id = odom_id_;
          t.child_frame_id = body_id_;
          t.transform.translation.x = odometry_msg.pose.pose.position.x;
          t.transform.translation.y = odometry_msg.pose.pose.position.y;
          t.transform.translation.z = 0.0;
          t.transform.rotation.x = odometry_msg.pose.pose.orientation.x;
          t.transform.rotation.y = odometry_msg.pose.pose.orientation.y;
          t.transform.rotation.z = odometry_msg.pose.pose.orientation.z;
          t.transform.rotation.w = odometry_msg.pose.pose.orientation.w;
          tf_broadcaster_odom_->sendTransform(t);

            // Also Publish the Blue Odometry uncorrected by SLAM:
          geometry_msgs::msg::TransformStamped t_blue;
          t_blue.header.stamp = msg->header.stamp;
          t_blue.header.frame_id = "nusim/world";
          t_blue.child_frame_id = "blue/base_footprint";
          t_blue.transform.translation.x = diff_drive_.get_q().translation().x;
          t_blue.transform.translation.y = diff_drive_.get_q().translation().y;
          t_blue.transform.translation.z = 0.0;
          t_blue.transform.rotation = t.transform.rotation;
          tf_broadcaster_odom_->sendTransform(t_blue);
        });

        // Initialize Reset service
    reset_service_ = this->create_service<nuturtle_control_interfaces::srv::InitialPose>(
                "~/initial_pose",
                std::bind(&SlamOdometry::handle_service_initial_pose, this, std::placeholders::_1,
      std::placeholders::_2)
    );
  }

private:
    /// \brief Resets the odometry to think the robot is at the requested configuration.
    ///
    /// Resets the internal odometry state to make the robot think it is at the configuration specified by service call.
    ///
    /// \param request - The requested initial pose for the robot to reset to, in the form of a geometry_msgs/Pose message.
    /// \param response - bool success value indicating whether the reset was successful.
    /// \returns void
  void handle_service_initial_pose(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Initial Pose Reset!");
        // Reset the internal odometry state with the requested configuration:
    diff_drive_.set_q(turtlelib::Transform2D(turtlelib::Vector2D{request->pose.position.x,
        request->pose.position.y}, request->pose.orientation.z));
        // Return success for reset:
    response->success = true;
  }

    /// \brief Creates visualization markers for the SLAM map obstacles:
    ///
    /// \returns A MarkerArray containing the predicted positions of the obstacles in the SLAM map based on the EKF SLAM algorithm.
  visualization_msgs::msg::MarkerArray createSLAMObstacles(
    std::vector<turtlelib::Point2D> slam_obstacles)
  {
    // Initialize and publish the Fake Obstacles:
    visualization_msgs::msg::MarkerArray marker_array_fake_obstacles;

    for (size_t i = 0; i < slam_obstacles.size(); ++i) {
      // Select each obstacle Point:
      auto obs = slam_obstacles.at(i);
      // Initialize a marker for each obstacle:
      visualization_msgs::msg::Marker marker;

      // Publish the markers with respect to the world frame, but check against the robots positon.
      // ############################## Begin_Citation [13] ##################################
      marker.header.frame_id = "nusim/world";
      // ############################### End_Citation [13] ###################################
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "slam_obstacles";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // Set orientation
      marker.pose.orientation.w = 1.0;

      // Determine obstacle locations and size:
      double obs_x = obs.x;
      double obs_y = obs.y;
      double obs_r = obstacle_radius_; // I set this to the actual size for our model at this point.
      // Set the marker information for the fake obstacles:
      marker.pose.position.x = obs_x;
      marker.pose.position.y = obs_y;
      marker.pose.position.z = obstacle_height_ / 2;
      marker.scale.x = obs_r * 2;
      marker.scale.y = obs_r * 2;
      marker.scale.z = obstacle_height_ + 0.02; // Add a little extra height to make it visible in RViz above the Ground Truth Obstacles
      // Fake Obstacles (Green)
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      // Add obstacle to marker array
      marker_array_fake_obstacles.markers.emplace_back(marker);
    }
    return marker_array_fake_obstacles;
  }

    // Initialize ROS 2 Infrustructure:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstacles_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_slam_;

    // Initialize the Parameters for Odometry:
  std::string body_id_ = declare_parameter<std::string>("body_id", "green/base_footprint");
  std::string odom_id_ = declare_parameter<std::string>("odom_id", "odom");
  std::string wheel_left_ = declare_parameter<std::string>("wheel_left");
  std::string wheel_right_ = declare_parameter<std::string>("wheel_right");
  double wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
  double track_width_ = declare_parameter<double>("track_width", 0.16);

    // Initialize Parameters for the slam obstacles:
  double obstacle_height_ = declare_parameter<double>("obst_height", 0.25);
  double obstacle_radius_ = declare_parameter<double>("slam_obs_radius", 0.038);

    // Initialize the DiffDrive model for internal odometry state:
  turtlelib::DiffDrive diff_drive_{track_width_, wheel_radius_};

  // Initialize the Parameters for the SLAM EKF:
  double slam_process_variance_ = declare_parameter<double>("slam_process_variance", 0.01);
  double slam_measurement_variance_ = declare_parameter<double>("slam_measurement_variance", 0.01);
  int num_obstacles_ = declare_parameter<int>("num_obstacles", 5);
  double mahalanobis_threshold = declare_parameter<double>("mahalanobis_threshold", 5.9);

  // Create the EKF SLAM object:
  slamlib::EKF slam_ekf_{0, slam_process_variance_, slam_measurement_variance_};  // Initialize the map wit no known obstacles.

  // Transform2D to hold the current estimated pose of the robot based on the EKF SLAM algorithm:
  turtlelib::Transform2D q_slam_;
  // Vector of landmarks in the SLAM map based on the EKF SLAM algorithm:
  std::vector<turtlelib::Point2D> landmarks_slam_;  // Point of where each landmark is in the map based on the EKF SLAM algorithm.
  // Path message to hold the current path of the robot based on the EKF SLAM algorithm:
  nav_msgs::msg::Path slam_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamOdometry>());
  rclcpp::shutdown();
  return 0;
}
