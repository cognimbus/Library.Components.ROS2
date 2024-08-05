from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def generate_launch_description():

    # args that can be set from the command line or a default will be used
    rtsp_url = DeclareLaunchArgument(
        "rtsp_url",
        default_value="rtsp://127.0.0.1:8554/video",
        description=''
    )

    width = DeclareLaunchArgument(
        "width",
        default_value="1920",
        description=''
    )

    height = DeclareLaunchArgument(
        "height",
        default_value="1080",
        description=''
    )
    max_buffers = DeclareLaunchArgument(
        "max_buffers",
        default_value="1",
        description=''
    )
    rtsp_node = Node(
            package='gst_rtsp',
            executable='gst_rtsp_node',
            name='gst_rtsp_node',
            parameters=[{
                "rtsp_url": LaunchConfiguration('rtsp_url'),
                "width":LaunchConfiguration('width'),
                "height":LaunchConfiguration('height'),
                "max_buffers":LaunchConfiguration('max_buffers')
            }]
        )

    return LaunchDescription([
        rtsp_url,
        width,
        height,
        max_buffers,
        rtsp_node,
    ])