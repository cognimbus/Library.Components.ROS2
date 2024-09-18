
FROM ultralytics/ultralytics:latest

ARG ROS_EDITION=humble
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC


RUN apt update && \
    update-alternatives --install /usr/bin/python3 python3 /opt/conda/bin/python3 1 && \
    apt install -y locales  && \
    locale-gen en_US en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    if [ "$ROS_EDITION" = "foxy" ]; then \
            apt install -y ros-foxy-ros-base python3-argcomplete; \
            # Action A
        elif [ "$ROS_EDITION" = "humble" ]; then \
            apt install -y ros-humble-ros-base; \
            # Action B
        else \
            exit 0 ; \
            # Default action
        fi && \
    apt install -y python3-colcon-common-extensions   && \
    apt install -y libboost-all-dev && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime  && \
    dpkg-reconfigure -f noninteractive tzdata  && \
    apt-get update && apt-get install ros-$ROS_EDITION-cv-bridge ros-$ROS_EDITION-librealsense2 -y && \
    apt-get install ros-$ROS_EDITION-message-filters ros-$ROS_EDITION-image-transport -y && \
    apt-get install ros-$ROS_EDITION-realsense2-camera -y && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    apt update && apt install -y ros-$ROS_EDITION-librealsense2* && \
    apt install -y python3-rosdep && \
    rosdep init && rosdep update && \
    pip install -U openvino && \
    pip3 install catkin_pkg

WORKDIR /upcreate3_object_detection_ws/src

RUN git clone -b $ROS_EDITION https://github.com/ros-perception/vision_opencv.git && \
    git clone -b galactic https://github.com/gbr1/upcreate3 && \
    git clone -b galactic https://github.com/gbr1/upcreate3_robot && \
    rm -rf /upcreate3_object_detection_ws/src/upcreate3_robot/upcreate3_navigation

COPY ./src /upcreate3_object_detection_ws/src
COPY ./yolov8n_openvino_model /upcreate3_object_detection_ws/yolov8n_openvino_model
COPY yolov8n.pt /upcreate3_object_detection_ws/
COPY data.json /upcreate3_object_detection_ws/
COPY object_detection.launch.py /upcreate3_object_detection_ws/src/upcreate3_robot/upcreate3_bringup/launch/object_detection.launch.py

WORKDIR /upcreate3_object_detection_ws

RUN . /opt/ros/$ROS_EDITION/setup.sh && rosdep install --from-paths src --ignore-src -y
RUN . /opt/ros/$ROS_EDITION/setup.sh && colcon build --symlink-install --packages-select cv_bridge image_geometry upcreate3_bringup upcreate3_control upcreate3_description upcreate3_robot yolo_detector

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]