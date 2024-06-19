#!/bin/bash

# Change directory to /src/razor-imu-ros2-humble/
cd ./src/razor-imu-ros2-humble/

# Use rosdep to install dependencies
rosdep install --from-paths razor_imu_ros2/ --ignore-src -y -r

sleep infinity