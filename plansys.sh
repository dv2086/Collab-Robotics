#!/bin/sh
cd /opt
mkdir -p plansys_ws/src
cd plansys_ws/src
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git
# plansys dependencies
git clone https://github.com/fmrico/popf.git -b ros2
git clone https://github.com/fmrico/cascade_lifecycle.git
cd ..
rosdep install -y -r -q --from-paths src --ignore-src
colcon build --symlink-install
. ./install/setup.sh