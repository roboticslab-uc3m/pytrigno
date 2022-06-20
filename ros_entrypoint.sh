#!/bin/bash

set -e
source /opt/ros/foxy/setup.bash 
colcon build
source /home/dev_ws/ros2/install/setup.bash

ros2 run delsys_publisher delsys_pub