# Development environment for the S10 perception racing stack.
#
# The contest mandates Ubuntu 24.04 + ROS 2 Jazzy, which ros:jazzy-ros-base provides.
# The image is architecture-neutral: the SDK ships prebuilt ONNX Runtime for both x86_64
# and aarch64, so this builds and runs natively on Apple Silicon as well as on an x86 box.
# Pick the matching value with the BUILD_PLATFORM argument to scripts/build.sh.

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-yaml \
      libevdev-dev \
      ros-jazzy-geometry-msgs \
      ros-jazzy-nav-msgs \
      ros-jazzy-sensor-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-rosidl-default-generators \
      libgl1 \
      libglx-mesa0 \
      libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

# numpy is pinned below 2.0 by the SDK README; MuJoCo is the simulator itself.
# scipy is imported by the upstream simulator but missing from the README's list.
# Ubuntu 24.04 marks the system Python as externally managed, hence the override.
RUN pip3 install --no-cache-dir --break-system-packages \
      "numpy<2.0" \
      mujoco \
      scipy

# Software rasteriser: containers have no GPU, and the race runs headless anyway
# (S10_USE_VIEWER=0). Set MUJOCO_GL=egl instead when a GPU is passed through.
ENV MUJOCO_GL=osmesa

# Both simulator and controller must share a DDS domain to see each other.
ENV ROS_DOMAIN_ID=1

WORKDIR /ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]
