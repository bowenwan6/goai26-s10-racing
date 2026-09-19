# syntax=docker/dockerfile:1
#
# S10 ROS 1 gateway build/test image (linux/arm64, the same Ubuntu 24.04 as the
# 48 AGX).
#   ROS 2 : Jazzy from the official ros:jazzy-ros-base image (pinned digest)
#   ROS 1 : ROS-O "one" packages for noble (Noetic line, roscpp 1.17.3), because
#           upstream Noetic has no Ubuntu 24.04 build.
#   bridge: ros2/ros1_bridge master at a pinned commit, built from source.
#
# Build (from ros1_gateway/):
#   docker build --platform linux/arm64 -f docker/Dockerfile.builder -t s10-ros1-gateway-builder .
#
# This image is for building and testing only. It is never run on the robot;
# the AGX gets a relocatable ROS 1 bundle and builds the gateway natively
# against its own Jazzy (see scripts/agx_build.sh).

FROM ros:jazzy-ros-base@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8 AS ros1base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# packages.ros.org is not reachable from the build network; the base image
# already contains the Jazzy packages needed here. ports.ubuntu.com answered
# 502 through this network, so an HTTPS mirror is used; apt still verifies the
# Ubuntu archive signatures.
ARG UBUNTU_MIRROR=https://mirrors.aliyun.com/ubuntu-ports
RUN rm -f /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.list \
 && sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}|g" /etc/apt/sources.list.d/ubuntu.sources

# ROS-O archive key, fingerprint 1B801DEE443DADF53C1A517E971CE6653AD9FC37
# (Robert Haschke, ros-o maintainer). The build fails if the file changes.
COPY docker/ros-one-archive-keyring.gpg /etc/apt/keyrings/ros-one-archive-keyring.gpg
RUN echo "af390037cf59b6b12db876fa1e47a177ce3fb31b1b726413726f23caa22aba3d  /etc/apt/keyrings/ros-one-archive-keyring.gpg" | sha256sum -c - \
 && echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/ros-one-archive-keyring.gpg] https://ros.packages.techfak.net noble main" \
      > /etc/apt/sources.list.d/ros-one.list

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-one-roscpp ros-one-rospy ros-one-rosbag ros-one-rostopic ros-one-rosmaster \
      ros-one-rosout ros-one-rosnode ros-one-rosmsg ros-one-roslaunch ros-one-rosparam \
      ros-one-sensor-msgs ros-one-nav-msgs ros-one-geometry-msgs ros-one-std-msgs \
      ros-one-std-srvs ros-one-rosgraph-msgs ros-one-topic-tools \
      libyaml-cpp-dev pkg-config git python3-numpy \
 && rm -rf /var/lib/apt/lists/* \
 && dpkg-query -W -f='${Package}\t${Version}\n' 'ros-one-*' 'ros-jazzy-rclcpp' 'ros-jazzy-rmw-fastrtps-cpp' \
      > /opt/package-versions.tsv

FROM ros1base AS bridge
ARG ROS1_BRIDGE_COMMIT=611755fd917285316051cbea80507e8b2f6b7ec1
ARG BUILD_JOBS=4
WORKDIR /ws
RUN git clone https://github.com/ros2/ros1_bridge.git src/ros1_bridge \
 && git -C src/ros1_bridge checkout --quiet ${ROS1_BRIDGE_COMMIT} \
 && git -C src/ros1_bridge log -1 --format='ros1_bridge %H %cd' > /opt/ros1_bridge.commit
# ROS 1 first, then ROS 2, as ros1_bridge's README requires. Only /opt/ros/jazzy
# is sourced on the ROS 2 side, so the generated factories cover just the
# message packages present in both installs.
RUN source /opt/ros/one/setup.bash && source /opt/ros/jazzy/setup.bash \
 && MAKEFLAGS=-j${BUILD_JOBS} colcon build --packages-select ros1_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --event-handlers console_direct- \
 && source install/setup.bash \
 && ros2 run ros1_bridge dynamic_bridge --print-pairs > /opt/ros1_bridge.pairs.txt 2>&1 || true

FROM bridge AS gateway
COPY src/s10_ros1_gateway /ws/src/s10_ros1_gateway
COPY src/drdds /ws/src/drdds
COPY src/s10_ros1_control /ws/src/s10_ros1_control
RUN source /opt/ros/one/setup.bash && source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
 && colcon build --packages-select s10_ros1_gateway drdds s10_ros1_control \
      --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
