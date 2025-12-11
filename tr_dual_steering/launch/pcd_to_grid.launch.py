import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # [수정] 본인의 PCD 파일 경로로 변경하세요
    pcd_path = '/home/home/RCI_dual_steering_ws/src/tr_dual_steering/pcd/test.pcd' 

    return LaunchDescription([
        # 1. PCD 파일 발행 (여전히 base_link로 나갈 수 있음)
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_publisher',
            output='screen',
            parameters=[{
                'file_name': pcd_path,
                'interval': 5.0,
                'frame_id': 'map',  # 설정해도 무시될 수 있음 -> 아래 3번 노드로 해결
                'latch': True
            }]
        ),

        # 2. Octomap Server (표준 패키지 사용)
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'frame_id': 'map',
                'base_frame_id': 'base_footprint',
                
                # 높이 필터링 (바닥 노이즈 제거 + 천장 제거)
                'sensor_model/max_range': 100.0,
                'occupancy_min_z': 0.1,    
                'occupancy_max_z': 1.5,
                
                # 들어오는 데이터가 map이 아니라 base_link여도 변환 가능하게 함
                'filter_ground': False 
            }],
            remappings=[
                ('cloud_in', '/cloud_pcd'),
                ('projected_map', '/map')
            ]
        ),

        # 3. [핵심 해결책] map과 base_link를 강제로 연결하는 TF 발행
        # (x, y, z, yaw, pitch, roll, frame_id, child_frame_id)
        # 이렇게 하면 pcd_publisher가 base_link로 쏴도 octomap이 map 기준이라 생각하고 받아들임
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        )
    ])