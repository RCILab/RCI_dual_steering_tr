from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tr_description_share = get_package_share_directory('tr_description')


    config_arg = DeclareLaunchArgument(
        'config',
        default_value='0',
        description='Select config: aligned or diagonal'
    ),
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ),


    config = LaunchConfiguration('config'),
    use_sim_time = LaunchConfiguration('use_sim_time')


    if config not in ('0', '1'):
        raise RuntimeError(f"[kinematics.launch] invalid config='{config}', use 0 or 1")
    yaml_config_file = PythonExpression(["'config/aligned.yaml' if '", config, "' == '0' else 'config/diagonal.yaml'"])
    yaml_path = os.path.join(tr_description_share, yaml_config_file)
    xacro_file = os.path.join(tr_description_share, 'robots', 'dual_steering_robot.urdf.xacro')


    # xacro $(ros2 pkg prefix tr_description)/share/tr_description/robots/dual_steering_robot.urdf.xacro yaml_path:=$(ros2 pkg prefix tr_kinematics_2ws)/share/tr_kinematics_2ws/config/aligned.yaml > src/tr_description/urdf/dual_steering_robot.urdf
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_gazebo': 'true',
                  'yaml_path': yaml_path,}
    ).toxml()


    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }],
        )
    ])