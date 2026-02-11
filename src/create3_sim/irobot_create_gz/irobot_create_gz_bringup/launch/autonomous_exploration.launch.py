"""
Autonomous Exploration Launch File

Launches the full navigation stack (via create3_navigation.launch.py)
and adds explore_lite for autonomous frontier-based exploration.

The robot will automatically discover and map the environment.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_create3_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')

    # Launch Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI).'
    )

    # 1. Full navigation stack (Gazebo + SLAM + Nav2 + RViz)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_create3_gz_bringup,
                'launch',
                'create3_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # 2. Explore Lite - autonomous frontier-based exploration
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
            'planner_frequency': 0.5,               # Replan every 2s
            'progress_timeout': 20.0,               # Abandon unreachable frontiers quickly
            'potential_scale': 0.5,                  # Low value = explore more evenly, avoid nearby dead-ends
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.25,               # Catch narrow corridor openings
            'return_to_init': False,                 # Don't stop when frontiers temporarily empty
        }]
    )

    # 3. Exploration Watchdog (recovery when stuck)
    watchdog_script_path = '/home/gokturk/create3_sim_ws/src/create3_sim/irobot_create_gz/irobot_create_gz_bringup/scripts/exploration_watchdog.py'
    watchdog_process = ExecuteProcess(
        cmd=['python3', watchdog_script_path, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # Sequencing: navigation stack starts first, then exploration after Nav2 is fully active
    # Nav2 starts at ~70s in create3_navigation.launch.py, needs ~30s to activate
    return LaunchDescription([
        headless_arg,
        navigation_launch,
        TimerAction(period=120.0, actions=[explore_node]),
        TimerAction(period=125.0, actions=[watchdog_process]),
    ])
