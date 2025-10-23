import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def robot_state_publisher_spawner(context, cfg):
    cfg_str = cfg.perform(context) # '0' or '1'
    if cfg_str not in ('0', '1'):
        raise RuntimeError(f"[kinematics.launch] invalid config='{cfg_str}', use 0 or 1")

    pkg_share = FindPackageShare('tr_dual_steering').find('tr_dual_steering')
    gazebo_ros_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    gazebo_world = PathJoinSubstitution([pkg_share, 'world', 'test.world'])
    yaml_map = {'0': 'config/aligned.yaml','1': 'config/diagonal.yaml'}
    yaml_path = PathJoinSubstitution([pkg_share, yaml_map[cfg_str]])
    xacro_file = PathJoinSubstitution([pkg_share, 'robots', 'dual_steering_robot.urdf.xacro'])


    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'yaml_path:=', yaml_path,
    ])
    robot_description = {
        'use_sim_time': True,
        'robot_description': robot_description_cmd
    }


    # Gazebo set
    set_gazebo_env = [
        UnsetEnvironmentVariable('GAZEBO_RESOURCE_PATH'),
        UnsetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=f"{os.path.expanduser('~')}/.gazebo/models:"
                  "/home/home/RCI_dual_steering_ws/src/RCI_dual_steering_tr/tr_dual_steering/models"
        ),
    ]
    
    return [
        *set_gazebo_env,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={
                'world': gazebo_world,
                'verbose': 'true',
                'use_sim_time': 'true'
            }.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        # ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_frequency': 50.0,   # 필요시 조정
                'source_list': ['robot_qd_wheel_front_drive_joint',
                                'robot_qd_wheel_front_steer_joint',
                                'robot_qd_wheel_rear_drive_joint',
                                'robot_qd_wheel_rear_steer_joint']  # 특정 조인트만 노출하고 싶으면 사용
            }],
        ),
        Node(
            package='tr_dual_steering',
            executable='dual_steering_kinematics_node',
            name='tr_dual_steering',
            output='screen',
            parameters=[yaml_path],
        ),
        TimerAction(
            period=2.0,  # 필요시 1~3초 조정
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'tr', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
                    output='screen'
                )
            ]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'config',
            default_value='0',
            description='Select config: aligned or diagonal'
        ),

        OpaqueFunction(
            function=robot_state_publisher_spawner,
            args=[LaunchConfiguration('config')]
        ),

    ])