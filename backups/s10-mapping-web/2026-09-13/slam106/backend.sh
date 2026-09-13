#!/bin/bash
set -e
export PYTHONFAULTHANDLER=1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
exec nice -n 10 /usr/bin/python3 /home/user/s10_mapping_web/robot_backend.py
