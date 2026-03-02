# Nuslam  Description

## Launching the Simulator

The package includes a launch file to start the simulator with RViz to simulate EKF SLAM with Known Data Associated Obstacles:

* `ros2 launch nuslam  slam.launch.xml robot:=nusim cmd_src:=teleop` to see the robot in rviz with the walls and obstacles with teleop control.
* `ros2 service call /nusimulator/reset std_srvs/srv/Empty` to reset the turtle simulation.

* `NOTE: This simulation is currently not working. I there is a ping-pong position updating issue with the SLAM state.`


## `robot:=nusim` (Simulation Mode)

Runs the full simulation using:

- `nusim/nusimulator`
- `nuturtle_control/turtle_control`
- `nuslam/slam_odom`
- Fake obstacle generation
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
- Robot description loaders (red, blue, green)

---

## `robot:=localhost` (Physical Robot Mode)

Runs the stack on a real TurtleBot3 connected locally.

### Nodes Launched

- `nuturtle_control/odometry`
- `nuturtle_control/turtle_control`
- `numsr_turtlebot`
- `hlds_laser_publisher`
- `circle` or `teleop`