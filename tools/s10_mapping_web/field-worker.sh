#!/bin/bash
set -eo pipefail
# ROS-generated setup files probe optional variables and are not nounset-safe.
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
export PYTHONFAULTHANDLER=1 PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -u /home/user/s10_mapping_web/field_worker.py
