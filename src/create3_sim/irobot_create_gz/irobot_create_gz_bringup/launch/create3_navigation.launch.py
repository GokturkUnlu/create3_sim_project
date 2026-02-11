"""
Navigation Launch File - Manual Goal Navigation (No Autonomous Exploration)

Launches:
- Gazebo simulation with Create3 robot
- SLAM Toolbox for mapping
- Nav2 navigation stack
- RViz2 for visualization

Use with nav2_goal_sender.py to send goals manually.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_create3_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Path to our custom tuned params file
    tuned_params_file = '/home/gokturk/create3_sim_ws/src/create3_sim/irobot_create_gz/irobot_create_gz_bringup/config/nav2_params_tuned.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = LaunchConfiguration('world', default='maze')

    # Launch Arguments
    headless_arg = DeclareLaunchArgument(
        'headless', 
        default_value='false',
        description='Run Gazebo in headless mode (no GUI).'
    )

    # 1. Gazebo Simulation with Robot
    create3_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_create3_gz_bringup,
                'launch',
                'create3_gz.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_arg,
            'use_rviz': 'false',
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # 2. SLAM Toolbox (for building a map while navigating)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_create3_gz_bringup, 'launch', 'create3_slam.launch.py'])
        )
    )

    # 3. Auto-Activate SLAM
    activate_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )
    activate_slam_cmd2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    # 4. CMD_VEL Relay (bridges Nav2 velocity commands to Gazebo)
    relay_script_path = '/home/gokturk/create3_sim_ws/src/create3_sim/irobot_create_gz/irobot_create_gz_bringup/scripts/cmd_vel_relay.py'
    relay_process = ExecuteProcess(
        cmd=['python3', relay_script_path, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 5. Nav2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': tuned_params_file
        }.items()
    )

    # Sequencing: Start components with delays to ensure proper initialization
    # Gazebo model loading can take 60+ seconds on first run (downloading resources)
    return LaunchDescription([
        headless_arg,
        create3_gz,
        TimerAction(period=30.0, actions=[slam_launch]),
        TimerAction(period=50.0, actions=[activate_slam_cmd]),
        TimerAction(period=55.0, actions=[activate_slam_cmd2]),
        TimerAction(period=60.0, actions=[relay_process]),
        TimerAction(period=70.0, actions=[nav2_launch]),
        
        # 6. Optimized RViz for Navigation
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_create3_gz_bringup, 'rviz', 'create3_nav.rviz'])],
            output='screen'
        ),
    ])
