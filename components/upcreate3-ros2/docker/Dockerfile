FROM ros:galactic

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

WORKDIR /dev_ws/src
RUN ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime  && \    
    dpkg-reconfigure -f noninteractive tzdata  && \
    apt-get update && apt-get install ros-galactic-cv-bridge ros-galactic-librealsense2 -y && \
    apt-get install ros-galactic-message-filters ros-galactic-image-transport -y && \
    apt-get install ros-galactic-realsense2-camera -y && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    apt update && apt install -y ros-galactic-librealsense2* && \
    git clone -b galactic https://github.com/gbr1/upcreate3 && \
    git clone -b galactic https://github.com/gbr1/upcreate3_robot

COPY bringup.launch.py /dev_ws/src/upcreate3_robot/upcreate3_bringup/launch/bringup.launch.py
WORKDIR /dev_ws 

RUN rosdep install --from-paths src --ignore-src -y
RUN . /opt/ros/galactic/setup.sh && colcon build --symlink-install


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]