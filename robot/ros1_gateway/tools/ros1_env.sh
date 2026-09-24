# Source this file to use the user-space ROS 1 bundle (ROS-O "one", Noetic line).
# It only changes the environment of the current shell; nothing is installed.
# Do not source it in the same shell as a ROS 2 setup unless you are building
# or running the gateway (which needs both).
_S10_ROS1_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_S10_SYS="$_S10_ROS1_DIR/sysroot"
_S10_ONE="$_S10_SYS/opt/ros/one"
_S10_LIB="$_S10_SYS/usr/lib/aarch64-linux-gnu"

export ROS_DISTRO=one ROS_VERSION=1 ROS_PYTHON_VERSION=3
export ROS_ROOT="$_S10_ONE/share/ros"
export ROS_ETC_DIR="$_S10_ONE/etc/ros"
export ROS_PACKAGE_PATH="$_S10_ONE/share"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export CMAKE_PREFIX_PATH="$_S10_ONE${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="$_S10_ONE/lib/pkgconfig:$_S10_LIB/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="$_S10_ONE/lib:$_S10_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LIBRARY_PATH="$_S10_ONE/lib:$_S10_LIB${LIBRARY_PATH:+:$LIBRARY_PATH}"
export CPATH="$_S10_ONE/include:$_S10_SYS/usr/include${CPATH:+:$CPATH}"
export PYTHONPATH="$_S10_ONE/lib/python3/dist-packages:$_S10_SYS/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$_S10_ONE/bin:$_S10_SYS/usr/bin${PATH:+:$PATH}"
unset _S10_SYS _S10_ONE _S10_LIB
