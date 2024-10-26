from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # 경로 수정
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('path_planning'),  # 패키지 이름
        'config',
        'config.yaml'  # yaml 파일 이름
    )

    return LaunchDescription([
        Node(
            package='path_planning',  # 패키지 이름
            executable='path_planning',  # 여기에서 실행 파일 이름이 CMakeLists.txt와 일치하는지 확인
            name='path_planning',
            output='screen',
            parameters=[config_file_path]  # yaml 파일 경로
        )
    ])
