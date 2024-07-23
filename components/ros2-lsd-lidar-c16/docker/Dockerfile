FROM ros:humble

SHELL ["/bin/bash", "-c"]


RUN apt-get update && \ 
    apt-get -y install python3-pip && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y ros-humble-pcl-ros ros-humble-pluginlib ros-humble-pcl-conversions && \
    apt-get install -y libpcap-dev

WORKDIR /c16_ws/src

RUN git clone -b C16_V3.0 https://github.com/Lslidar/Lslidar_ROS2_driver.git
COPY lslidar_c16_launch.py /c16_ws/src/Lslidar_ROS2_driver/lslidar_driver/launch/lslidar_c16_launch.py
WORKDIR /c16_ws

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]