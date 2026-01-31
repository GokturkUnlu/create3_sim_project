# Create 3 Simulation Project

This repository contains the simulation setup for the iRobot Create 3, including performance optimizations and SLAM (Simultaneous Localization and Mapping) capabilities.

## Prerequisites

-   **ROS 2 Distribution**: Jazzy
-   **Simulator**: Gazebo Harmonic (gz-sim)
-   **Dependencies**:
    -   `ros_gz_bridge`
    -   `slam_toolbox`
    -   `irobot_create_gz_bringup` (and dependencies)

## Installation & Build

1.  **Clone the repository** (if not already done):
    ```bash
    git clone https://github.com/GokturkUnlu/create3_sim_project.git
    cd create3_sim_project
    ```

2.  **Build the workspace**:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## Usage Guide

To run the full simulation stack, you will need **5 Terminal Tabs**.

### Terminal 1: Simulation
Launches Gazebo and spawns the robot in the maze world.
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py world:=maze
```
**Expected Output**: Gazebo GUI opens. The robot appears in the maze. `RTF` (Real Time Factor) in the bottom right should be close to `1.00`.

### Terminal 2: ROS-GZ Bridge
Bridges the Lidar data from Gazebo to ROS 2.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```
**Expected Output**: Log indicating `Creating GZ->ROS Bridge`.

### Terminal 3: Teleoperation (Optional)
Allows you to drive the robot with the keyboard.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Instructions**: Follow the on-screen keys (e.g., `i` to move forward, `j`/`l` to turn).

### Terminal 4: SLAM (Mapping)
Starts the SLAM Toolbox to generate a map from the Lidar data covers the custom parameters for simulation latency.
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch irobot_create_gz_bringup create3_slam.launch.py
```
**Expected Output**: `[slam_toolbox]: Node using stack size...`. No errors about `unconfigured` state.

### Terminal 5: Visualization (RViz)
Visualizes the robot and the map.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

## RViz Configuration

1.  **Fixed Frame**: Set `Fixed Frame` (top left) to `map`.
2.  **Add Map**:
    -   Click **Add** (bottom left).
    -   Select **By Topic**.
    -   Find `/map` -> **Map**.
    -   Click **OK**.
    -   *(You should see the grey/white/black map grid appearing)*.
3.  **Add Robot Model**:
    -   Click **Add**.
    -   Select **RobotModel**.
4.  **Add LaserScan** (Optional):
    -   Click **Add**.
    -   Select **By Topic** -> `/scan` -> **LaserScan**.

## Saving the Map
Once you have driven around and are happy with the map:
```bash
ros2 run nav2_map_server map_saver_cli -f my_maze_map
```

## Troubleshooting
-   **"No map received"**: Ensure Terminal 4 (SLAM) is running and Terminal 2 (Bridge) is running.
-   **Laggy Control**: Ensure `cmd_vel_timeout` is set to 5.0 (already configured in this repo).
