## How to Run the Simulation
To run the full simulation with the Lidar active, you must open **three separate terminal tabs** and run the commands in order.

### Terminal 1: The Simulation
This launches Gazebo, the robot model, and the environment.

```bash
cd ~/create3_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py world:=maze
```
### Terminal 2: The Bridge (Vital for Lidar)
Gazebo generates the laser data, but ROS 2 cannot see it until you bridge the topic. Run this to connect them.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```
### Terminal 3: The Transform Fix
Gazebo and ROS use slightly different names for the Lidar's location. This command forces them to agree so the data appears in the right place in RViz.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 rplidar_link create3/base_link/lidar
```

