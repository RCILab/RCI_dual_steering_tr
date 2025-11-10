from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

import os
import xacro
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, cfg, use_sim_time_arg):
    cfg_str = cfg.perform(context) # '0' or '1'
    if cfg_str not in ('0', '1'):
        raise RuntimeError(f"[kinematics.launch] invalid config='{cfg_str}', use 0 or 1")

    pkg_share = get_package_share_directory('tr_dual_steering')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'tr.rviz')
    yaml_map = {'0': 'config/aligned.yaml','1': 'config/diagonal.yaml'}
    yaml_path = os.path.join(pkg_share, yaml_map[cfg_str])
    xacro_file = os.path.join(pkg_share, 'robots', 'dual_steering_robot.urdf.xacro')
    lidar_macro_path = os.path.join(pkg_share, 'robots', 'xacros', 'gazebo_lidar.xacro')

    # xacro $(ros2 pkg prefix tr_dual_steering)/share/tr_dual_steering/robots/dual_steering_robot.urdf.xacro yaml_path:=$(ros2 pkg prefix tr_dual_steering)/share/tr_dual_steering/config/aligned.yaml > dual_steering_robot.urdf
    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'yaml_path:=', yaml_path, ' ',
    ])
    robot_description = ParameterValue(
        robot_description_cmd,
        value_type=str
    )

    # robot_description = xacro.process_file(
    #     mappings={'use_gazebo': 'true', 'controller_yaml_path': yaml_path, 'lidar_macro_path': lidar_macro_path}
    # ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'use_sim_time': use_sim_time_arg
        }.items()
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_arg,
            'robot_description': robot_description
        }],
    )
    kinematics_node = Node(
        package='tr_dual_steering',
        executable='tr_dual_steering_node',
        name='tr_dual_steering_node',
        output='screen',
        parameters=[yaml_path],
    )
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tr', '-topic', '/robot_description'],
        output='screen'
    )
    spawn_jsb_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    delay_velocity_controller_spawner= RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb_node,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['velocity_controller', '-c', '/controller_manager'],
                    output='screen',
                )
            ]
        )
    )
    delay_trajectory_controller_spawner= RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb_node,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
                    output='screen',
                )
            ]
        )
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=(['-d', rviz_cfg] if rviz_cfg else [])
    )
    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        kinematics_node,
        spawn_jsb_node,
        delay_velocity_controller_spawner,
        delay_trajectory_controller_spawner,
        rviz_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='0',
            description='Select config: aligned or diagonal'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        OpaqueFunction(
            function=launch_setup,
            args=[
                LaunchConfiguration('config'),
                LaunchConfiguration('use_sim_time'),
            ]
        ),
    ])