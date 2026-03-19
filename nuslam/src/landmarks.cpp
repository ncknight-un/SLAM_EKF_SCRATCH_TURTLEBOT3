/// \file
/// \brief Creates a ROS 2 Node which allows the turtlebot to sense landmarks for SLAM sensor measurements.
///
/// PARAMETERS:
///     
/// PUBLISHES:
///     ~ real_obstacles ()
/// SUBSCRIBES:
///     ~/real_scan (sensor_msgs::msg::LaserScan): Lser scan data based on the robot's position and obstacles in the arena using LiDAR Sensor.
/// SERVERS:
///     ~ 
/// CLIENTS:
///     None

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <armadillo>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "slamlib/circle_reg.hpp"

/// \brief A class to launch a Landmarks locator Node
class Landmarks : public rclcpp::Node {
public:
  Landmarks()
  : Node("landmarks")
  {
    // Construct the publisher for SLAM obstacles:
    real_obstacle_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/scan_landmarks", 10);

    // Construct the subscriber for the real scaned data: 
    real_sensor_subscriber_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan_data", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // Break apart the sensor information using unsupervised learning to cluster the points based on the the dist_threhold for the sensor data.
            auto timestamp_ = msg->header.stamp; // timestamp of the scan
            auto frame_id_ = msg->header.frame_id; // frame id of the scan
            auto ranges_ = msg->ranges; // vector of ranges
            auto angle_min_ = msg->angle_min; // minimum angle of the scan
            auto angle_max_ = msg->angle_max; // maximum angle of the scan
            auto angle_increment_ = msg->angle_increment; // angle increment of the scan
            // auto range_min_ = msg->range_min; // minimum range of the scan
            auto range_max_ = msg->range_max; // maximum range of the scan

            // Initialize an arrray of clusters to hold the clustered points:
            std::vector<std::vector<size_t>> clusters; // Store the indices of the points in each cluster.

            // Scan through the ranges and cluster the points based on the distance threshold:
            for (size_t i = 0; i < ranges_.size(); i++) {
                // Get the range and angle of the current scan point:go chom
                auto range = ranges_.at(i);
                auto angle = angle_min_ + i * angle_increment_;
                
                if(angle > angle_max_) {
                  angle = angle_max_; // Make sure that we don't over index the angle.
                }

                // Don't consider a point for a cluster if it is at the maximum range (i.e. no obstacle detected):
                if (range >= range_max_)
                    continue;
                else {
                  // Get current point in robot frame:
                    double x = range * std::cos(angle);
                    double y = range * std::sin(angle);
                    turtlelib::Point2D current_point(x, y);

                    // Check if the current point belongs to an existing cluster:
                    bool added_to_cluster = false;
                    // Loop through existing clusters and see if the current point is within the distance to a point in that cluster:
                    for (auto & cluster : clusters) {
                        // Get the last point in the cluster:
                        auto last_index = cluster.back();
                        auto last_range = ranges_.at(last_index);
                        auto last_angle = angle_min_ + last_index * angle_increment_;
                        // Get the last point in the cluster in robot frame:
                        double last_x = last_range * std::cos(last_angle);
                        double last_y = last_range * std::sin(last_angle);
                        turtlelib::Point2D last_point(last_x, last_y);

                        // If the current point is within the distance threshold of the last point in the cluster, add it to the cluster. Otherwise continue to next cluster:
                        turtlelib::Vector2D diff = current_point - last_point;
                        auto distance = turtlelib::magnitude(diff);
                        if (distance < dist_threshold_) {
                            cluster.push_back(i);
                            added_to_cluster = true;
                            break;
                        }
                    }

                    RCLCPP_DEBUG_STREAM(this->get_logger(), "There are currently " << clusters.size() << " clusters detected.");

                    // If the current point was not added to any existing cluster, create a new cluster with this point:
                    if (!added_to_cluster) {
                        clusters.push_back({i});
                    }

                    // If we are at the end of the scan, need to check if the current point belongs to the first cluster to account for wrap around:
                    if((i == ranges_.size() - 1) && !clusters.empty()) {
                      // Get the first index in the first cluster:
                        auto first_index = clusters.front().front();
                        auto first_range = ranges_.at(first_index);
                        auto first_angle = angle_min_ + first_index * angle_increment_;
                        // Get the first point in the first cluster in robot frame:
                        double first_x = first_range * std::cos(first_angle);
                        double first_y = first_range * std::sin(first_angle);
                        turtlelib::Point2D first_point(first_x, first_y);

                        turtlelib::Vector2D diff = current_point - first_point;
                        auto distance = turtlelib::magnitude(diff);
                        if (distance < dist_threshold_) {
                          // Combine the first and laster cluster as they are within the distance threshold of one another:
                          clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
                          clusters.pop_back(); // Remove the last cluster as it is now combined with the first cluster
                        }
                    }
                } 
            }

            // Remove clusters that are too small to be an obstacle (i.e. less than or equal to min_cluster_size_):
            clusters.erase(
              std::remove_if(clusters.begin(), clusters.end(),
                [this](const std::vector<size_t>& cluster) {
                  return static_cast<int>(cluster.size()) <= min_cluster_size_;
                }),
              clusters.end()
            );

            RCLCPP_DEBUG_STREAM(this->get_logger(), "There are currently " << clusters.size() << " clusters detected after removing small clusters.");

            // Implement the circle fitting algorithm to find the centroid and radius of each cluster to determine the location of the obstacles:
            // for each cluster, build a vector of points and pass it to the circle fitting algorithm:
            // Build a vector of points in the current cluster:
            std::vector<turtlelib::Point2D> real_obstacle_centroids;
            std::vector<double> real_obstacle_radii;

            for (auto & cluster : clusters) {
                // Build a vector of points for the current cluster:
                std::vector<turtlelib::Point2D> cluster_points;
                for (auto & index : cluster) {
                    auto range = ranges_.at(index);
                    auto angle = angle_min_ + index * angle_increment_;
                    double x = range * std::cos(angle);
                    double y = range * std::sin(angle);
                    cluster_points.push_back(turtlelib::Point2D(x, y));
                }

                // Calculate the centroid and radius for this cluster using the circle fitting algorithm:
                auto [centroid, radius, is_circle] = slamlib::CircleReg(cluster_points).fitCircle();

                RCLCPP_DEBUG_STREAM(this->get_logger(), "Cluster has " << cluster_points.size() << " points, is_circle: " << is_circle << ", radius: " << radius << " (min: " << min_obstacle_radius_ << ", max: " << max_obstacle_radius_ << ")");

                // If it is a circle and within radius limits, keep it as a real obstacle:
                if (is_circle) {
                    if (radius >= min_obstacle_radius_ && radius <= max_obstacle_radius_) {
                      // Make sure that the centroid is not too close to another centroid already detected:
                        bool too_close = false;
                        for (const auto& existing_centroid : real_obstacle_centroids) {
                          // Calculate the distance between the existing centroid and the new centroid:
                          auto dist = turtlelib::magnitude(existing_centroid - centroid);
                          if (dist < min_centroid_dist_) {
                            too_close = true;
                            break;
                          }
                        }
                        if (!too_close) {
                          // Centroid is not too close to existing centroids, add it to the list of real obstacles:
                          real_obstacle_centroids.push_back(centroid);
                          real_obstacle_radii.push_back(radius);
                        }
                    }
                }
            }

            // For every centroid marked, publish a marker for visualization in RViz:
            visualization_msgs::msg::MarkerArray marker_array_real_obstacles;

            for (size_t i = 0; i < real_obstacle_centroids.size(); ++i) {
              // Initialize a marker for each obstacle:
              visualization_msgs::msg::Marker marker;

              // Publish the markers with respect to the world frame:
              marker.header.frame_id = frame_id_;
              marker.header.stamp = timestamp_;
              marker.ns = "real_obstacles";
              marker.id = i;
              marker.type = visualization_msgs::msg::Marker::CYLINDER;
              marker.action = visualization_msgs::msg::Marker::ADD;
              // Set orientation
              marker.pose.orientation.w = 1.0;

              // Determine obstacle locations and size:
              auto obs = real_obstacle_centroids.at(i);
              double obs_r = real_obstacle_radii.at(i);
              // Set the marker information for the real obstacles:
              marker.pose.position.x = obs.x;
              marker.pose.position.y = obs.y;
              marker.pose.position.z = obstacle_height_ / 2;
              marker.scale.x = obs_r * 2;
              marker.scale.y = obs_r * 2;
              marker.scale.z = obstacle_height_ + 0.02; // Add a little extra height to make it visible in RViz above the Ground Truth Obstacles
              // Real Obstacles (Green)
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0;
              marker.color.a = 1.0;
              // Add obstacle to marker array
              marker_array_real_obstacles.markers.emplace_back(marker);
            }
            // Publish the marker array:
            real_obstacle_pub_->publish(marker_array_real_obstacles);
        });
  }

private:
  // Initialize the ROS Infrastructure:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr real_sensor_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr real_obstacle_pub_;

  // Initialize the parameters for Unserpervised Learning Clustering:
  double dist_threshold_ = declare_parameter<double>("dist_threshold", 0.1); // Distance threshold for clustering points together (0.1m)
  double min_obstacle_radius_ = declare_parameter<double>("min_obstacle_radius", 0.05); // Minimum radius for a cluster to be considered a real obstacle (0.05m)
  double max_obstacle_radius_ = declare_parameter<double>("max_obstacle_radius", 0.5); // Maximum radius for a cluster to be considered a real obstacle (0.5m)
  int min_cluster_size_ = declare_parameter<int>("min_cluster_size", 3); // Minimum number of points for a cluster to be considered a real obstacle (3 points)
  // Initialize the parameter for the height of the obstacles for visualization in RViz:
  double obstacle_height_ = declare_parameter<double>("obstacle_height", 0.25);
  double min_centroid_dist_ = declare_parameter<double>("min_centroid_dist", 0.05); // Minimum distance between centroids to be visualized as separate obstacles (0.05m)
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}