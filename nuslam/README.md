# Nuslam  Description

## Launching the Simulator

The package include the following launch files: 

1) A launch file to start the simulator with RViz to simulate EKF SLAM with Known Data Associated Obstacles:

* `ros2 launch nuslam  slam.launch.xml robot:=nusim cmd_src:=teleop` to see the robot in rviz with the walls and obstacles with teleop control.
* `ros2 service call /nusimulator/reset std_srvs/srv/Empty` to reset the turtle simulation.

2) A launch file to start the circle fitting algorithm in simulation to mark the landmarks.
* `ros2 launch nuslam landmark_detect.launch.xml robot:=nusim cmd_src:=teleop` to test the circle fitting algorithm on the simulated sensor data. 

2) A launch file to start the simulator with RViz to simulate EKF SLAM with Unknown Data Associated Obstacles:
* `ros2 launch nuslam unknown_data_assoc.launch.xml robot:=nusim cmd_src:=teleop` to test the unknown data association detection of the obstacles with EKF SLAM.

## Teleop Simulation:
##### INSERT SIM VIDEO HERE

### Error Analsis:
Final pose error Ground truth -> Odometry:
Ground Truth (0.001, 0.040)
Odometry (-0.017, -0.010)
`Positional Error`:  0.0531 m (5.31 cm)

Final pose error Ground truth -> SLAM-EKF:
Ground Truth (0.001, 0.040)
SLAM-EKF(0.000, 0.042)
`Positional Error`: 0.00224 m (0.224 cm)

## Circle Drive Simulation:
##### INSERT SIM VIDEO HERE


## `robot:=nusim` (Simulation Mode)

Runs the full simulation using:

- `nusim/nusimulator`
- `nuturtle_control/turtle_control`
- `nuslam/slam_odom`
- `nuslam/landmarks`
- Lidar Sensor Uknown Data Associated Obstacle Generation
- Multi-robot visualization (Red, Blue, Green bots)

### Robot Roles in Simulation

| Robot | Purpose |
|--------|----------|
| **Red**   | Ground truth simulation robot |
| **Blue**  | Pure odometry robot |
| **Green** | SLAM-estimated robot |

### Nodes Launched

- **RViz2** (optional)
- `nusimulator`
- `turtle_control`
- `slam_odom`
- `circle` or `teleop`

---

## `robot:=localhost` (Physical Robot Mode)

Runs the stack on a real TurtleBot3 connected locally.

### Nodes Launched

- `nuturtle_control/odometry`
- `nuturtle_control/turtle_control`
- `numsr_turtlebot`
- `hlds_laser_publisher`
- `circle` or `teleop`