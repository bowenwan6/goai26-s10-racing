# Development environment for the S10 perception racing stack.
#
# The contest mandates Ubuntu 24.04 + ROS 2 Jazzy, which ros:jazzy-ros-base provides.
# The image is architecture-neutral: the SDK ships prebuilt ONNX Runtime for both x86_64
# and aarch64, so this builds and runs natively on Apple Silicon as well as on an x86 box.
# Pick the matching value with the BUILD_PLATFORM argument to scripts/build.sh.

# Pin the multi-architecture image index used for the accepted 2026-08-19 run.  Keeping the
# human-readable Jazzy tag makes the platform obvious; the digest prevents a later rebuild from
# silently changing ROS or Ubuntu underneath the submitted code.
FROM ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923

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

# NumPy must remain below 2.0 for the SDK; MuJoCo is the simulator itself.
# scipy is imported by the upstream simulator but missing from the README's list.
# onnxruntime runs the shipped policy outside the C++ controller, so a manoeuvre can be
# designed against the same network the deployment uses; the SDK bundles only the C++ API.
# Ubuntu 24.04 marks the system Python as externally managed, hence the override.
COPY requirements.lock /tmp/s10-requirements.lock
RUN pip3 install --no-cache-dir --break-system-packages \
      --requirement /tmp/s10-requirements.lock \
    && rm /tmp/s10-requirements.lock

# Software rasteriser: containers have no GPU, and the race runs headless anyway
# (S10_USE_VIEWER=0). Set MUJOCO_GL=egl instead when a GPU is passed through.
ENV MUJOCO_GL=osmesa

# Both simulator and controller must share a DDS domain to see each other.
ENV ROS_DOMAIN_ID=1

# Build artefacts live outside the bind-mounted repo; see docker/run.sh for why.
ENV S10_BUILD_BASE=/opt/s10-build/build
ENV S10_INSTALL_BASE=/opt/s10-build/install
ENV S10_LOG_BASE=/opt/s10-build/log

WORKDIR /ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f "${S10_INSTALL_BASE}/setup.bash" ] && source "${S10_INSTALL_BASE}/setup.bash"' >> /root/.bashrc

CMD ["/bin/bash"]
