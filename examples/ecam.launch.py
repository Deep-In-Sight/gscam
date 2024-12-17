from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # fisrt camera node 첫 번째 카메라 노드 실행
        Node(
            package='gscam',
            executable='gscam_node',
            namespace='ecam_left',
            name='gscam_driver',
            parameters=[{
                'camera_name': 'default',
                'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                'gscam_config': 'nvarguscamerasrc sensor-id=0 sensor-mode=3 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12 ! nvvidconv ! video/x-raw,format=NV12 ! videoconvert',
                'frame_id': '/base_link',
                'sync_sink': True
            }],
            output='screen'
        ),
        # second camera node after 2 seconds 2초 지연 후 두 번째 카메라 노드 실행
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gscam',
                    executable='gscam_node',
                    namespace='ecam_middle',
                    name='gscam_driver',
                    parameters=[{
                        'camera_name': 'default',
                        'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                        'gscam_config': 'nvarguscamerasrc sensor-id=1 sensor-mode=3 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12 ! nvvidconv ! video/x-raw,format=NV12 ! videoconvert',
                        'frame_id': '/base_link',
                        'sync_sink': True
                    }],
                    output='screen'
                )
            ]
        ),
        # second camera node after 2 more seconds 추가 2초 지연 후 세 번째 카메라 노드 실행
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gscam',
                    executable='gscam_node',
                    namespace='ecam_right',
                    name='gscam_driver',
                    parameters=[{
                        'camera_name': 'default',
                        'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                        'gscam_config': 'nvarguscamerasrc sensor-id=2 sensor-mode=3 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12 ! nvvidconv ! video/x-raw,format=NV12 ! videoconvert',
                        'frame_id': '/base_link',
                        'sync_sink': True
                    }],
                    output='screen'
                )
            ]
        )
    ])
