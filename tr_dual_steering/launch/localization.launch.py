import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declare_pcd_map = DeclareLaunchArgument('pcd_map', 
                                            default_value= '/home/home/RCI_dual_steering_ws/src/tr_dual_steering/pcd/vlp16.pcd',
                                            description='Full path to map(pcd)file')
    declare_yaml_map = DeclareLaunchArgument('yaml_map', 
                                            default_value= '/home/home/RCI_dual_steering_ws/src/tr_dual_steering/map/vlp16.yaml',
                                            description='Full path to map(yaml)file')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_params = DeclareLaunchArgument('params_file', 
                                           default_value= '/home/home/RCI_dual_steering_ws/src/tr_dual_steering/config/velodyne16_localization.yaml',
                                           description= 'Full path to navigation yaml file')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true') # [수정] 추가됨
    
    pcd_map_path = LaunchConfiguration('pcd_map')
    yaml_map_path = LaunchConfiguration('yaml_map')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    

    tr_dual_steering_share = get_package_share_directory('tr_dual_steering')
    default_rviz_cfg = os.path.join(tr_dual_steering_share, 'rviz', 'localization.rviz')
    default_localization_config = os.path.join(tr_dual_steering_share, 'config', 'velodyne16_localization.yaml')


    lifecycle_nodes = ['map_server']

    remappings = [('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/cmd_vel', '/cmd_vel_raw')]
    
    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename': yaml_map_path}
    
    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            default_localization_config,  
            {
                'use_sim_time': use_sim_time, # [수정] Sim Time 적용
                'feature_extract_enable': False,
                'point_filter_num': 4,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'pcd_save_enable': False,
            }
        ],
    )
    
    global_localization_node = Node(
        package='fast_lio',
        executable='global_localization.py',
        name='global_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/map', '/map_cloud'),
        ]
    )

    transform_fusion_node = Node(
        package='fast_lio',
        executable='transform_fusion.py',
        name='transform_fusion',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    initial_pose_node = Node(
        package='fast_lio',
        executable='publish_initial_pose.py',
        name='publish_initial_pose',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=[
            '-0.013', '0.0', '0.49',  # x y z
            '0.0', '0.0', '0.0'       # yaw pitch roll (rad)
        ],
    )

    map_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='map_publisher',
        output='screen',
        parameters=[{   
                'use_sim_time': use_sim_time,
                'file_name': pcd_map_path,   # 불러올 PCD 파일 경로
                'tf_frame': 'pcd_map',       # 퍼블리시되는 포인트클라우드의 frame_id
                'publishing_period_ms': 200,
        }],
        remappings=[
            ('cloud_pcd', '/map_cloud'),      # output topic을 /map 으로
        ],
    )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', default_rviz_cfg,
                '--ros-args', '-p', ['use_sim_time:=', use_sim_time]
            ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )
    

    ld = LaunchDescription()
    ld.add_action(declare_pcd_map)
    ld.add_action(declare_yaml_map)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params)
    ld.add_action(declare_namespace)
    ld.add_action(declare_autostart)

    


    ld.add_action(map_server_node)
    ld.add_action(fastlio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(map_publisher_node)
    ld.add_action(initial_pose_node)
    ld.add_action(rviz_node)
    ld.add_action(lifecycle_manager_node)

    return ld