FROM ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /rtsp_ws
COPY ./src /rtsp_ws/src


RUN apt-get update && apt-get upgrade -y && \ 
    apt-get -y install python3-pip && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio && \
    apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev && \
    apt-get install -y libopencv-dev python3-opencv && \
    apt-get install -y ros-humble-cv-bridge && \
    apt-get install -y build-essential && \
    pip install opencv-python && \
    rosdep install -i --from-path src --ignore-src -y

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]