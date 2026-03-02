# ME495 Sensing, Navigation and Machine Learning For Robotics
* Nolan Knight
* Winter 2025

### EKF SLAM Simulation Test #1: (NOTE: Current issue with position updating)


https://github.com/user-attachments/assets/03fdd3da-590c-4f56-9c46-5e33046bf6b5

`Next steps to fix this issue are to continue debugging node to determine if it is a message issue or if the algorithm is not implemented correctly`

### Odometry Turtlebot Test:

https://github.com/user-attachments/assets/654c35ba-8415-4926-86aa-6737d10a04a6


*** The odometry error was found to be: 
``` 
frame_id: odom
child_frame_id: blue/base_footprint
pose:
  pose:
    position:
      x: 0.25489205409876425
      y: -0.2402227613119873
      z: 0.0
    orientation:
      x: -0.0
      y: 0.0
      z: 0.6842614435150383
      w: -0.7292367769927104
  covariance:
  - 0.0
```

# Package List
This repository consists of several ROS packages
- Nuturtle-description - URDF development and Rviz vizualization of one or multiple turtlebots in one simulation.
- Turtlelib - Transfrom2D calculations for Point2D, Vector2D, and Twist2Ds. SVG visualization of transforms and transform calculations.
- Nusim - Ground Truth Simulation for TurtleRobot. Loads obstacles and world size based on config file of choice.
- Nuturtle_control - Odometry and Diff_Drive implementation for Simulation and Localhost operation on turtle bot. Turtlebot can be controlled via a `circle node` or `teleop_keyboard`.
- Nuturtle_control_interfaces - Custom service message types for nuturtle_control package.
