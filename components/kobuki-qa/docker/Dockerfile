FROM ros:humble

RUN  apt-get update && apt-get install ros-humble-cv-bridge ros-humble-librealsense2 -y && \
    apt-get install ros-humble-message-filters ros-humble-image-transport -y && \
    apt-get install ros-humble-realsense2-camera -y
    
RUN apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    rosdep install -i --from-path src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /kobuki_ws/src
RUN git clone --branch=humble https://github.com/IntelligentRoboticsLabs/kobuki.git
RUN git clone https://github.com/Hokuyo-aut/urg_node2.git

WORKDIR /kobuki_ws
RUN . /opt/ros/humble/setup.sh && rosdep install -i --from-path src --ignore-src -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]