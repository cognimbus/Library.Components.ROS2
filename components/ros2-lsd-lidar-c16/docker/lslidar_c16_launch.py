#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():

    declare_arguments = [
        DeclareLaunchArgument('frame_id', default_value='laser_link', description='The frame id of the lidar.'),
        DeclareLaunchArgument('device_ip', default_value='192.168.1.200', description="The lidar's IP address."),
        DeclareLaunchArgument('msop_port', default_value='2368', description="The MSOP port."),
        DeclareLaunchArgument('difop_port', default_value='2369', description="The DIFOP port."),
        DeclareLaunchArgument('add_multicast', default_value='false', description="Add multicast."),
        DeclareLaunchArgument('group_ip', default_value='224.1.1.2', description="The group IP address."),
        DeclareLaunchArgument('use_gps_ts', default_value='false', description="Use GPS timestamp."),
        DeclareLaunchArgument('lidar_type', default_value='c16', description="Lidar type."),
        DeclareLaunchArgument('c16_type', default_value='c16_2', description="C16 type."),
        DeclareLaunchArgument('c32_type', default_value='c32_2', description="C32 type."),
        DeclareLaunchArgument('c32_fpga_type', default_value='3', description="C32 FPGA type."),
        DeclareLaunchArgument('min_range', default_value='0.3', description="Minimum range."),
        DeclareLaunchArgument('max_range', default_value='200.0', description="Maximum range."),
        DeclareLaunchArgument('distance_unit', default_value='0.25', description="Distance unit."),
        DeclareLaunchArgument('angle_disable_min', default_value='0', description="Minimum disable angle."),
        DeclareLaunchArgument('angle_disable_max', default_value='0', description="Maximum disable angle."),
        DeclareLaunchArgument('scan_num', default_value='8', description="Number of scans."),
        DeclareLaunchArgument('horizontal_angle_resolution', default_value='0.18', description="Horizontal angle resolution."),
        DeclareLaunchArgument('packet_rate', default_value='840.0', description="Packet rate."),
        DeclareLaunchArgument('topic_name', default_value='lslidar_point_cloud', description="Topic name."),
        DeclareLaunchArgument('publish_scan', default_value='true', description="Publish scan."),
        DeclareLaunchArgument('pcl_type', default_value='false', description="PCL type."),
        DeclareLaunchArgument('coordinate_opt', default_value='false', description="Coordinate optimization.")
    ]

    param = {
        "packet_size": 1206,  # 雷达数据包长度(字节)，填1212/1206
        "device_ip": LaunchConfiguration('device_ip'),
        "msop_port": LaunchConfiguration('msop_port'),
        "difop_port": LaunchConfiguration('difop_port'),
        "frame_id": LaunchConfiguration('frame_id'),
        "add_multicast": LaunchConfiguration('add_multicast'),
        "group_ip": LaunchConfiguration('group_ip'),
        "use_gps_ts": LaunchConfiguration('use_gps_ts'),
        "lidar_type": LaunchConfiguration('lidar_type'),  # c16表示机械式16线雷达；c32表示机械式32线雷达
        "c16_type": LaunchConfiguration('c16_type'),  # c16_2表示16线垂直角度分辨率为2度的雷达，c16_1表示16线垂直角度分辨率为1.33度的雷达
        "c32_type": LaunchConfiguration('c32_type'),  # c32_2表示32线垂直角度分辨率为1度的雷达，c32_1表示32线垂直角度分辨率为0.33度的雷达
        "c32_fpga_type": LaunchConfiguration('c32_fpga_type'),  # 3表示32线fpga为2.7\2.8\3.0的版本的雷达，2表示32线fpga为2.6的版本的雷达
        "min_range": LaunchConfiguration('min_range'),
        "max_range": LaunchConfiguration('max_range'),
        "distance_unit": LaunchConfiguration('distance_unit'),
        "angle_disable_min": LaunchConfiguration('angle_disable_min'),
        "angle_disable_max": LaunchConfiguration('angle_disable_max'),
        "scan_num": LaunchConfiguration('scan_num'),
        "horizontal_angle_resolution": LaunchConfiguration('horizontal_angle_resolution'),
        "packet_rate": LaunchConfiguration('packet_rate'),
        "topic_name": LaunchConfiguration('topic_name'),
        "publish_scan": LaunchConfiguration('publish_scan'),
        "pcl_type": LaunchConfiguration('pcl_type'),
        "coordinate_opt": LaunchConfiguration('coordinate_opt')
    }

    driver_node = LifecycleNode(
        package='lslidar_driver',
        namespace='c16',
        executable='lslidar_c16driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[param],
    )

    return LaunchDescription([*declare_arguments,driver_node])
