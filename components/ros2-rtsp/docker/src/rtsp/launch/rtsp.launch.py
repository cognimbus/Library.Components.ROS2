from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def generate_launch_description():

    # args that can be set from the command line or a default will be used
    url = DeclareLaunchArgument(
        "url",
        default_value="rtsp://127.0.0.1:8554/video",
        description=''
    )

    image_topic_name = DeclareLaunchArgument(
        "image_topic_name",
        default_value="rtsp_stream_image",
        description=''
    )

    nobuffer = DeclareLaunchArgument(
        "nobuffer",
        default_value="true",
        description=''
    )

    rtsp_node = Node(
            package='rtsp',
            executable='rtsp_node',
            name='rtsp_node',
            parameters=[{
                "url": LaunchConfiguration('url'),
                "image_topic_name":LaunchConfiguration('image_topic_name'),
                "nobuffer":LaunchConfiguration('nobuffer')
            }]
        )

    return LaunchDescription([
        url,
        image_topic_name,
        nobuffer,
        rtsp_node,
    ])