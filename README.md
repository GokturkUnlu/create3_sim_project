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

iRobot Create 3 Simulation & Autonomous Navigation
This repository contains the simulation, navigation, and autonomous exploration packages for the iRobot Create 3 in Gazebo. It supports two primary modes: Auto-Motion (sending specific coordinate goals) and Autonomous Exploration (frontier-based mapping).

Prerequisites
Before running any command, ensure you have sourced your ROS 2 environment (Jazzy) and the workspace overlay.

Bash
source /opt/ros/jazzy/setup.bash
source ~/create3_sim_ws/install/setup.bash
##Task 1: Auto-Motion (Point-to-Point Navigation)
Use this mode to launch the simulation and send the robot to a specific (x, y) coordinate using the Nav2 stack.

Step 1: Launch Simulation & Navigation
Open a new terminal and run the main launch file. This starts Gazebo, Rviz, and the Navigation2 stack.

Bash
cd ~/create3_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch simulation and Nav2
ros2 launch irobot_create_gz_bringup create3_navigation.launch.py
Wait until you see "Nav2 is ready for use" or the Rviz window is fully loaded.

Step 2: Send a Goal
Open a second terminal to send a specific goal command.

Bash
cd ~/create3_sim_ws
source install/setup.bash

# Send the robot to x=2.0, y=1.0
python3 src/create3_sim/irobot_create_gz/irobot_create_gz_bringup/scripts/nav2_goal_sender.py --x 2.0 --y 1.0
You can change the --x and --y arguments to any valid coordinate on the map.

##Task 2: Autonomous Exploration
Use this mode to let the robot autonomously map an unknown environment using frontier exploration (SLAM).

Open a terminal and run the all-in-one launch file:

Bash
cd ~/create3_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch Autonomous Exploration (Sim + SLAM + Exploration Node)
ros2 launch irobot_create_gz_bringup autonomous_exploration.launch.py
The robot will detect unknown areas (frontiers) and automatically navigate to them to build a complete map of the environment.

##Troubleshooting
"Package not found": Make sure you ran colcon build and source install/setup.bash in the current terminal.

Robot not moving: Ensure the simulation isn't paused in Gazebo.

Navigation fails: Check if the goal coordinates (--x, --y) are inside a valid map area (not inside a wall).
