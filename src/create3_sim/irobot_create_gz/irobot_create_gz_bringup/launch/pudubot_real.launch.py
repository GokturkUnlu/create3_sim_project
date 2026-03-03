"""
PuduBot Real Hardware Launch File

Brings up the full SLAM + Nav2 stack using REAL sensor data
from a PuduBot (HLS2/BellaBot) connected via USB/ADB.

No Gazebo simulation — lidar data comes from LD06 via ADB exec-out.
Odometry is derived from laser scan matching (rf2o) or SLAM Toolbox.

Prerequisites:
  1. PuduBot connected via USB (ADB authorized)
  2. rf2o_laser_odometry (optional):
     sudo apt install ros-jazzy-rf2o-laser-odometry

Usage:
  ros2 launch irobot_create_gz_bringup pudubot_real.launch.py
  ros2 launch irobot_create_gz_bringup pudubot_real.launch.py use_rf2o:=false
  ros2 launch irobot_create_gz_bringup pudubot_real.launch.py adb_serial:=52D3602B1341763
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('irobot_create_gz_bringup')

    # Use absolute path to scripts dir (same as existing project convention)
    scripts_dir = os.path.join(
        os.path.expanduser('~'),
        'create3_sim_ws/src/create3_sim/irobot_create_gz/'
        'irobot_create_gz_bringup/scripts'
    )
    config_dir = os.path.join(
        os.path.expanduser('~'),
        'create3_sim_ws/src/create3_sim/irobot_create_gz/'
        'irobot_create_gz_bringup/config'
    )

    # ── Launch Arguments ──────────────────────────────────────────────────
    use_rf2o_arg = DeclareLaunchArgument(
        'use_rf2o', default_value='true',
        description='Use rf2o_laser_odometry for scan-matching odometry.'
    )
    adb_serial_arg = DeclareLaunchArgument(
        'adb_serial', default_value='52D3602B1341763',
        description='ADB device serial for the PuduBot.'
    )

    use_rf2o = LaunchConfiguration('use_rf2o')
    adb_serial = LaunchConfiguration('adb_serial')

    # ── 1. LD06 Lidar Driver (uses ADB exec-out internally) ──────────────
    ld06_driver = ExecuteProcess(
        cmd=[
            'python3', os.path.join(scripts_dir, 'ld06_driver_node.py'),
            '--ros-args',
            '-p', 'source:=adb',
            '-p', ['adb_serial:=', adb_serial],
            '-p', 'lidar_dev:=/dev/ttyS2',
            '-p', 'frame_id:=laser',
            '-p', 'range_min:=0.02',
            '-p', 'range_max:=12.0',
        ],
        output='screen'
    )

    # ── 2. Static TF: base_link → laser ──────────────────────────────────
    # LD06 is mounted at x=0.176m from base_link center (from PuduBot config)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '0.176', '0', '0.25',   # x, y, z offset
            '0', '0', '0',           # roll, pitch, yaw
            'base_link', 'laser'
        ]
    )

    # ── 3. Static TF: base_link → base_footprint ─────────────────────────
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # ── 4. Laser Odometry (rf2o) ──────────────────────────────────────────
    # Publishes odom → base_link transform and /odom topic
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
        condition=IfCondition(use_rf2o),
    )

    # Fallback: static odom → base_link when rf2o is not used
    # SLAM Toolbox will still localize via scan matching (map → odom)
    odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_link_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=UnlessCondition(use_rf2o),
    )

    # ── 5. SLAM Toolbox ──────────────────────────────────────────────────
    slam_params_file = os.path.join(config_dir, 'slam_params_pudubot.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # Auto-activate SLAM lifecycle
    activate_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )
    activate_slam_cmd2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    # ── 6. Nav2 Navigation Stack ──────────────────────────────────────────
    nav2_params_file = os.path.join(config_dir, 'nav2_params_pudubot.yaml')

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items()
    )

    # ── 7. RViz2 ──────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'create3_nav.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── Sequencing ────────────────────────────────────────────────────────
    # Real hardware boots faster than simulation, but still needs ordering
    return LaunchDescription([
        # Arguments
        use_rf2o_arg,
        adb_serial_arg,

        # Log
        LogInfo(msg='=== PuduBot Real Hardware Launch ==='),
        LogInfo(msg='Lidar data via ADB exec-out (no separate forwarder needed)'),

        # 1. Lidar driver + TFs (immediate)
        ld06_driver,
        lidar_tf,
        base_footprint_tf,
        odom_static_tf,  # only active when use_rf2o:=false

        # 2. Laser odometry (after 2s for /scan to be available)
        TimerAction(period=2.0, actions=[rf2o_node]),

        # 3. SLAM (after 5s)
        TimerAction(period=5.0, actions=[slam_node]),
        TimerAction(period=8.0, actions=[activate_slam_cmd]),
        TimerAction(period=10.0, actions=[activate_slam_cmd2]),

        # 4. Nav2 (after 15s)
        TimerAction(period=15.0, actions=[nav2_launch]),

        # 5. RViz (immediate)
        rviz,
    ])
