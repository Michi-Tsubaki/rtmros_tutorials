#!/bin/bash

source ~/catkin_ws/devel/setup.bash
rossetip
rossetmaster 133.11.216.115
source ~/colcon_ws/install/setup.bash
ros2 launch ros1bridge.launch.xml
