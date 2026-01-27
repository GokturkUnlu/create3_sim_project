iRobot Create 3 Simulation (ROS 2 Jazzy)
This repository allows you to run the iRobot Create 3 simulation in Gazebo Harmonic on Ubuntu 24.04.

📋 Prerequisites
OS: Ubuntu 24.04 (Noble Numbat)

ROS Version: ROS 2 Jazzy installed.

🚀 Setup Instructions
Run these commands one by one in your terminal to set up the project.

1. Clone the Repository
Bash
cd ~
git clone https://github.com/GokturkUnlu/create3_sim_project.git create3_sim_ws
2. Install Dependencies
This automatically installs the required Gazebo libraries and ROS drivers.

Bash
cd ~/create3_sim_ws
sudo apt update
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
3. Build the Project
Bash
colcon build --symlink-install
🎮 How to Run
To open the simulation, run these commands:

Bash
cd ~/create3_sim_ws
source install/setup.bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py
Note: The first time you run this, Gazebo might take 1-2 minutes to open while it downloads 3D models. Please be patient!
