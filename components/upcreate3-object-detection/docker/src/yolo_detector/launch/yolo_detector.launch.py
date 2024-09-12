from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def generate_launch_description():

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
            parameters=[{
                "threshold": LaunchConfiguration('threshold')
            }]
        )

    return LaunchDescription([
        threshold,
        yolo_detector_node
    ])