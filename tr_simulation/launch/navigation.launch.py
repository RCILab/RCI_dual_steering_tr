# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    tr_simulation =get_package_share_directory('tr_simulation')
    tr_navigation =get_package_share_directory('tr_navigation')


    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value='vlp16',
        description='Name of the map file in tr_real/maps directory (without .yaml extension)'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the stacks'
    )

    map_name = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(tr_simulation,'config','navigation.yaml')
    )

    # pcd_map_path = os.path.join(tr_simulation, 'pcd', map_name,'.pcd')
    # yaml_map_path = os.path.join(tr_simulation, 'maps', map_name,'.yaml')
    pcd_map_path = PathJoinSubstitution([
        tr_simulation, 
        'pcd', 
        PythonExpression(["'", map_name, ".pcd'"]) # 'map_name' + '.pcd'
    ])
    
    yaml_map_path = PathJoinSubstitution([
        tr_simulation, 
        'maps', 
        PythonExpression(["'", map_name, ".yaml'"]) # 'map_name' + '.yaml'
    ])

    return LaunchDescription([
        map_name_arg,
        autostart_arg,
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(tr_navigation, 'launch','localization_fastlio.launch.py')
            ]),
            launch_arguments={
                'pcd_map': pcd_map_path,
                'yaml_map' : yaml_map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'namespace': namespace,
                'autostart': autostart}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(tr_navigation, 'launch','navigation.launch.py')
            ]),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir,
                              'autostart': autostart}.items()
        ),
    ])