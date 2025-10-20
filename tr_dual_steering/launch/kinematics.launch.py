from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    cfg = LaunchConfiguration('config', default='aligned')
    config_map = {
        'aligned': 'config/aligned.yaml',
        'diagonal': 'config/diagonal.yaml'
    }
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='aligned',
                              description='Select config: aligned or diagonal'),
        Node(
            package='tr_dual_steering',
            executable='dual_steering_kinematics_node',
            name='tr_dual_steering',
            output='screen',
            parameters=[
                # 보통은 --params-file로 YAML을 넘기는 것을 권장합니다.
                # 여기서는 최소한의 동작만 기본 파라미터로 보장합니다.
            ],
        )
    ])
