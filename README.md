# ME495 Sensing, Navigation and Machine Learning For Robotics
* Nolan Knight
* Winter 2025

## Real-World EKF-SLAM:
`Still working through real world implementation problems. Will update videas when real world simulation is working.`

## Teleop No-Data_Association EKF-SLAM Simulation:

https://github.com/user-attachments/assets/83b06239-9bfb-47df-b908-b1e1cc987cfc

### Error Analsis:
Final pose error Ground truth -> Odometry:
Ground Truth (0.001, 0.040)
Odometry (-0.017, -0.010)
`Positional Error`:  0.0531 m (5.31 cm)

Final pose error Ground truth -> SLAM-EKF:
Ground Truth (0.001, 0.040)
SLAM-EKF(0.000, 0.042)
`Positional Error`: 0.00224 m (0.224 cm)

## Circle Drive No-Data_Association EKF-SLAM Simulation:

https://github.com/user-attachments/assets/a0d91412-18e1-445b-ac3c-9be6821b9612


### EKF SLAM Simulation with 5 Obstacles:


https://github.com/user-attachments/assets/8ed3b104-f71b-4a6c-a2d7-fa64cd3cb5de


`There is a slight delay in green tf publication. I will look into this at the start of HW4.`

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
- Slamlib - All algorithms for mapping and state estimation using in SLAM-EKF.
- Nusim - Ground Truth Simulation for TurtleRobot. Loads obstacles and world size based on config file of choice.
- Nuturtle_control - Odometry and Diff_Drive implementation for Simulation and Localhost operation on turtle bot. Turtlebot can be controlled via a `circle node` or `teleop_keyboard`.
- Nuturtle_control_interfaces - Custom service message types for nuturtle_control package.
