from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kinematics_share = get_package_share_directory('tr_kinematics_2ws')


    yaml_path_arg = DeclareLaunchArgument(
            'yaml_path',
            default_value=os.path.join(kinematics_share,'config','aligned.yaml'),
            description='Select config: aligned or diagonal'
        )
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    

    yaml_path = LaunchConfiguration('yaml_path'),
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        yaml_path_arg,
        use_sim_time_arg,


        Node(
            package='tr_kinematics_2ws',
            executable='tr_kinemetics_2ws_node',
            name='tr_kinemetics_2ws_node',
            output='screen',
            parameters=[yaml_path, {'use_sim_time': use_sim_time}],
        )
    ])