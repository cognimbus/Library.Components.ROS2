# The MIT License
# 
# Copyright (c) 2022 Giovanni di Dio Bruno https://gbr1.github.io
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    # upcreate3 folders
    upcreate3_control_path = get_package_share_directory('upcreate3_control')
    realsense_path = get_package_share_directory('realsense2_camera')



    # realsense launcher

    realsense_launch_file = PathJoinSubstitution(
        [realsense_path, 'launch', 'rs_launch.py'])
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments = {'align_depth.enable': 'true',
                            'initial_reset'     : 'true',
                            'pointcloud.enable' : 'false',
                            'rgb_camera.color_profile':'640x480x30'
                           }.items()
    )

    # twist_mux
    twist_mux_launch_file = PathJoinSubstitution(
        [upcreate3_control_path, 'launch', 'twist_mux.launch.py'] 
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_mux_launch_file)
    )

    # args that can be set from the command line or a default will be used
    threshold = DeclareLaunchArgument(
        "threshold",
        default_value="0.0",
        description=''
    )
    yolo_detector_node = Node(
            package='yolo_detector',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            remappings=[
                ('/camera/image_raw','/camera/camera/color/image_raw'),
            ],
            parameters=[{
                "threshold": LaunchConfiguration('threshold')
            }]
        )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(realsense_launch)
    ld.add_action(twist_mux_launch)
    ld.add_action(threshold)
    ld.add_action(yolo_detector_node)
    return ld