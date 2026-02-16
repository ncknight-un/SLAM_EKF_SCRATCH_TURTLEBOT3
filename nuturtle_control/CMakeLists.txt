cmake_minimum_required(VERSION 3.20)
project(nuturtle_control)

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

# Create Executables and Link Libraries:
add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

# Link the libraries to the executables:
target_link_libraries(turtle_control PUBLIC
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  turtlelib::turtlelib
)
target_link_libraries(odometry PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  sensor_msgs::sensor_msgs_library
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  rclcpp::rclcpp
)
target_link_libraries(circle PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  turtlelib::turtlelib
)

# Install Targets:
install(TARGETS
turtle_control
  circle
  odometry
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ############################################### Begin_Citation [11] #########################
  find_package(catch_ros2 REQUIRED)
  include(CTest)

  # Set each test executable's dependencies:
  add_executable(turtle_control_test test/turtle_control_test.cpp)
  target_link_libraries(turtle_control_test PRIVATE
    rclcpp::rclcpp
    catch_ros2::catch_ros2_with_node_main
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nuturtlebot_msgs::nuturtlebot_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  )

  install(TARGETS
    turtle_control_test
    DESTINATION lib/${PROJECT_NAME}
  )
  ############################################### End_Citation [11] #########################

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add the ros2 integration tests:
  catch_ros2_add_integration_test(turtle_control_test LAUNCH_FILE turtle_control_test.launch.xml)

  install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
)
endif()

# Set up Install Directories:
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()