#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
colcon build
source /home/dev_ws/ros2/install/setup.bash

ros2 run ros2_pytrigno delsys_publisher
