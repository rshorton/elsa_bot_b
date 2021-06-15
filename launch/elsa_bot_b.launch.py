import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    elsa_bot_b_dir = get_package_share_directory('elsa_bot_b')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(elsa_bot_b_dir, 'params', 'mymap_ds3.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(elsa_bot_b_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    # Create2 robot driver
    create = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('create_bringup'), 'launch', 'create_2.launch.py'))
    )

    # web bridge (for proxying topics/actions to/from ros_web based applications)
    pose_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch', 'robot_pose_publisher.launch.py'))
    )

    # Nav2
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'map': map_yaml_file,
                          'params_file': params_file}.items()
    )

    # ROS2 web bridge - run a bash script to run the nodejs based process
    web_bridge = Node(
        package='elsa_bot_b',
        executable='ros2_web_bridge_launch'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(create)
    ld.add_action(pose_pub)
    ld.add_action(nav)
    ld.add_action(web_bridge)

    return ld
