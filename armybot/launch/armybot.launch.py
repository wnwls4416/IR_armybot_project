from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 흑백 IR 카메라 기반 YOLO 인식 노드
        Node(
            package='armybot',
            executable='yoloon',
            name='yolo_vision_node',
            output='screen'
        ),
        
        # 2. 표적지 분석 AI 노드
        Node(
            package='armybot',
            executable='ai_opencv',
            name='target_analysis_node',
            output='screen'
        ),

        # 3. 로봇 메인 미션 제어 노드 (지휘관)
        Node(
            package='armybot',
            executable='robot_control',
            name='robot_mission_executor',
            output='screen'
        )
    ])