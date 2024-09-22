FROM ros:humble

RUN  apt-get update && apt-get install ros-humble-cv-bridge ros-humble-librealsense2 -y && \
    apt-get install ros-humble-message-filters ros-humble-image-transport -y && \
    apt-get install ros-humble-realsense2-camera -y && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
RUN . /opt/ros/humble/setup.sh

CMD ["ros2", "launch", "realsense2_camera", "rs_launch.py"]
