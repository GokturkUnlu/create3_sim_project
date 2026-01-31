from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_create3_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # pkg_explore_lite might not be in share if installed via apt, but it usually is. 
    # Safest is to rely on Node executable lookup.

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = LaunchConfiguration('world', default='maze')

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_create3_gz_bringup, 'launch', 'create3_gz.launch.py'])
        ),
        launch_arguments={'world': world_arg}.items()
    )

    # 2. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_create3_gz_bringup, 'launch', 'create3_slam.launch.py'])
        )
    )

    # 3. Auto-Activate SLAM (Workaround for lifecycle issue)
    activate_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )
    activate_slam_cmd2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    # 4. CMD_VEL Relay (The Fix for Robot Motion)
    # Using the script we created earlier
    relay_script_path = '/home/gokturk/create3_sim_ws/src/create3_sim/irobot_create_gz/irobot_create_gz_bringup/scripts/cmd_vel_relay.py'
    
    # We use ExecuteProcess to run the python script directly
    relay_process = ExecuteProcess(
        cmd=['python3', relay_script_path, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 5. Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. Explore Lite (Tuned)
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/global_costmap/costmap',
            'costmap_updates_topic': '/global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 60.0,    # Increased from 30
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.4,    # Decreased from 0.75
        }]
    )

    # Sequencing logic
    # - Gazebo starts (0s)
    # - SLAM starts (5s)
    # - SLAM activates (15s) - give it time to load
    # - Relay starts (20s)
    # - Nav2 starts (25s)
    # - Explore starts (40s) - wait for Nav2 to be fully active

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=5.0, actions=[slam_launch]),
        TimerAction(period=15.0, actions=[activate_slam_cmd]),
        TimerAction(period=18.0, actions=[activate_slam_cmd2]),
        TimerAction(period=20.0, actions=[relay_process]),
        TimerAction(period=25.0, actions=[nav2_launch]),
        TimerAction(period=40.0, actions=[explore_node]),
    ])
