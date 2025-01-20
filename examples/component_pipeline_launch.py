# Copyright 2022 Clyde McQueen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example pipeline using rclcpp_components.

This launches the gscam and other nodes into a container so that they run in the same process.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 카메라 노드 실행
        Node(
            package='gscam',
            executable='gscam_node',
            namespace='/camera/ecam_left',
            name='gscam_driver',
            parameters=[{
                'use_gst_timestamps': True,
                'camera_name': 'default',
                #camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                # 'gscam_config': 'nvarguscamerasrc sensor-id=0 sensor-mode=3 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12 ! nvvidconv ! video/x-raw,format=NV12 ! videoconvert',
                'gscam_config': 'nvarguscamerasrc sensor-id=0 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1  ! nvvidconv ! video/x-raw, width=640, height=538, format=NV12 ! videoconvert',
                #'gscam_config': 'nvarguscamerasrc sensor-id=0 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1  ! nvvidconv ! video/x-raw, format=NV12 ! videoconvert',
                'frame_id': '/base_link',
                'sync_sink': False
            }],
            output='screen'
        ),
        # run second camera node after 2 seconds delay 2초 지연 후 두 번째 카메라 노드 실행
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gscam',
                    executable='gscam_node',
                    namespace='/camera/ecam_middle',
                    name='gscam_driver',
                    parameters=[{
                        'use_gst_timestamps': True,
                        'camera_name': 'default',
                        #'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                        'gscam_config': 'nvarguscamerasrc sensor-id=2 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1 ! nvvidconv ! video/x-raw, width=640, height=538,format=NV12 ! videoconvert',
                        #'gscam_config': 'nvarguscamerasrc sensor-id=2 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1  ! nvvidconv ! video/x-raw, format=NV12 ! videoconvert',
                        'frame_id': '/base_link',
                        'sync_sink': False
                    }],
                    output='screen'
                )
            ]
        ),
        # run second camera node after 2 more seconds delay 추가 2초 지연 후 세 번째 카메라 노드 실행
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gscam',
                    executable='gscam_node',
                    namespace='/camera/ecam_right',
                    name='gscam_driver',
                    parameters=[{
                        'use_gst_timestamps': True,
                        'camera_name': 'default',
                        # 'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                        'gscam_config': 'nvarguscamerasrc sensor-id=1 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1  ! nvvidconv ! video/x-raw, width=640, height=538, format=NV12 ! videoconvert',
                        #'gscam_config': 'nvarguscamerasrc sensor-id=1 sensor-mode=1 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12, framerate=10/1  ! nvvidconv ! video/x-raw, format=NV12 ! videoconvert',
                        'frame_id': '/base_link',
                        'sync_sink': False
                    }],
                    output='screen'
                )
            ]
        )
    ])