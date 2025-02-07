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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # 멀티스레드 컨테이너 (또는 component_container 사용)
            composable_node_descriptions=[
                # 카메라 0 인스턴스
                ComposableNode(
                    package='gscam',      # gscam 패키지 이름
                    plugin='gscam::GSCam',
                    name='camera0',
                    parameters=[{
                        'gscam_config': "nvarguscamerasrc sensor-id=0 sensor-mode=0 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12 ! nvvidconv ! video/x-raw, width=720,height=606,format=NV12 ! videoconvert",
                        'camera_name': "camera0",
                        'camera_info_url': "package://gscam/examples/uncalibrated_parameters.ini",
                        'frame_id': "camera0_frame"
                    }]
                ),
                # 카메라 1 인스턴스
                ComposableNode(
                    package='gscam',
                    plugin='gscam::GSCam',
                    name='camera1',
                    parameters=[{
                        'gscam_config': "nvarguscamerasrc sensor-id=2 sensor-mode=0 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12 ! nvvidconv ! video/x-raw, width=720,height=606,format=NV12 ! videoconvert",
                        'camera_name': "camera1",
                        'camera_info_url': "package://gscam/examples/uncalibrated_parameters.ini",
                        'frame_id': "camera1_frame"
                    }]
                ),
                # 카메라 1 인스턴스
                ComposableNode(
                    package='gscam',
                    plugin='gscam::GSCam',
                    name='camera2',
                    parameters=[{
                        'gscam_config': "nvarguscamerasrc sensor-id=1 sensor-mode=0 ! video/x-raw(memory:NVMM),width=2432,height=2048,format=NV12 ! nvvidconv ! video/x-raw, width=720,height=606,format=NV12 ! videoconvert",
                        'camera_name': "camera2",
                        'camera_info_url': "package://gscam/examples/uncalibrated_parameters.ini",
                        'frame_id': "camera2_frame"
                    }]
                )
                # 추가 카메라 인스턴스도 필요에 따라 추가
            ],
            output='screen'
        )
    ])