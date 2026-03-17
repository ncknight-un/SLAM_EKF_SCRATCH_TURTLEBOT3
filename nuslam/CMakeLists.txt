cmake_minimum_required(VERSION 3.20)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtle_control_interfaces REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slamlib REQUIRED)

# Set target link libraries:
add_executable(slam_odom src/slam_odom.cpp)
target_link_libraries(slam_odom PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  slamlib::slamlib
  rclcpp::rclcpp
)
add_executable(landmarks src/landmarks.cpp)
target_link_libraries(landmarks PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  sensor_msgs::sensor_msgs_library
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  slamlib::slamlib
)

install(TARGETS
  slam_odom
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

# Execution of Unit Testing CTests:  (run ctest --verbose)
include(CTest)

option(BUILD_TESTING "Allow Unit Test Execution" ON)

if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)

  add_executable(circle_test tests/circle_test.cpp)
  target_link_libraries(circle_test Catch2::Catch2WithMain slamlib::slamlib turtlelib::turtlelib)

  # Register the test with CTest:
  add_test(Name TestCircle COMMAND circle_test)
endif()

# Set up Install Directories:
install(DIRECTORY
  launch
  config
  tests
  DESTINATION share/${PROJECT_NAME}
)

ament_package()