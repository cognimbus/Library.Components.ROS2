FROM ultralytics/ultralytics:latest-jetson-jetpack5

SHELL ["/bin/bash", "-c"]

WORKDIR /yolo_detector_ws
COPY ./src /yolo_detector_ws/src


RUN apt update && \ 
    apt install -y locales && \ 
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y  ros-foxy-ros-base && \
    apt install -y python3-colcon-common-extensions && \
    apt install libboost-all-dev

WORKDIR /yolo_detector_ws/src
RUN git clone -b foxy https://github.com/ros-perception/vision_opencv.git

WORKDIR /yolo_detector_ws
RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]