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
    yaml_map = {'0': 'config/aligned.yaml','1': 'config/diagonal.yaml'}

    tr_simulation_share = get_package_share_directory('tr_simulation')
    tr_description_share = get_package_share_directory('tr_description')
    tr_kinematics_share = get_package_share_directory('tr_kinematics_2ws')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    yaml_path = os.path.join(tr_kinematics_share, yaml_map[cfg_str])
    xacro_file = os.path.join(tr_description_share, 'robots', 'dual_steering_robot.urdf.xacro')
    world_path = os.path.join(tr_simulation_share,'worlds','costmap_test.world')
    
    # xacro $(ros2 pkg prefix tr_dual_steering)/share/tr_dual_steering/robots/dual_steering_robot.urdf.xacro yaml_path:=$(ros2 pkg prefix tr_dual_steering)/share/tr_dual_steering/config/aligned.yaml > dual_steering_robot.urdf
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_gazebo': 'true',
                  'yaml_path': yaml_path,}
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'use_sim_time': use_sim_time_arg
        }.items()
    )
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tr', '-topic', '/robot_description'],
        output='screen'
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
    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tr_kinematics_2ws'), 'launch', 'kinematics.launch.py')
        ),
        launch_arguments={
            'cofig': yaml_path,
            'use_sim_time': use_sim_time_arg,
        }.items()
    )

    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_jsb_node,
        delay_velocity_controller_spawner,
        delay_trajectory_controller_spawner,
        kinematics_launch
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