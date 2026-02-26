FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ament-cmake \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-serial-driver \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-apriltag-ros \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-serial \
    setserial \
    minicom \
    screen \
    net-tools \ 
    iproute2 \
    python3-pip \
    libserial-dev \
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    joystick \
    evtest \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install bleak

# Initialize ROS dependencies
RUN rosdep init && rosdep update || true

# Purge modemmanager to avoid conflicts
RUN apt-get purge -y modemmanager || true

 # Add dialout group and add root user to it
 RUN groupadd -f dialout && usermod -aG dialout root

# Set the RMW implementation to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Source ROS 2 in every interactive shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspaces/xgolite_ws


ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]