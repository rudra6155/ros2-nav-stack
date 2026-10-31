##############################################################################
# ROS 2 Humble + Gazebo Classic (gazebo11) development / evaluation image.
#
# All dependencies are installed at BUILD time.
##############################################################################
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG ROS_APT_URL=http://packages.ros.org/ros2/ubuntu
ARG ROS_GPG_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=42 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    GAZEBO_MODEL_DATABASE_URI="" \
    QT_X11_NO_MITSHM=1

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg2 locales tzdata; \
    locale-gen en_US.UTF-8; \
    curl -fsSL "${ROS_GPG_URL}" -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_URL} jammy main" \
        > /etc/apt/sources.list.d/ros2.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential cmake git procps psmisc util-linux jq \
        python3-pip python3-pytest python3-numpy python3-matplotlib python3-yaml \
        python3-colcon-common-extensions \
        ros-humble-ros-base \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-tools \
        ros-humble-message-filters ros-humble-std-srvs \
        ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
        ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
        ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
        ros-humble-rviz2 \
        xvfb x11-utils x11-xserver-utils libgl1-mesa-dri mesa-utils ffmpeg; \
    rm -rf /var/lib/apt/lists/*

# Login shells (bash -lc ...) get ROS, Gazebo and the workspace overlay.
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh' \
    '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' \
    'export PATH=/workspace/scripts:$PATH' \
    > /root/.bash_profile

WORKDIR /workspace/ros2_ws
CMD ["bash", "-l"]
