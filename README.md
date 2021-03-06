# Collab-Robotics
>Group24: Conner Moore cm4841@nyu.edu, Dhruv Vajpeyi dv2086@nyu.edu, Shikun Cui sc8186@nyu.edu
## Running the Python Planner
- Run the container

sudo docker run -it \                                                         
                --env="DISPLAY" \
                --env="QT_X11_NO_MITSHM=1" \
                --env="GAZEBO_MODEL_PATH=/opt/warehouse_ws/src/amazon_robot_gazebo/models:/opt/warehouse_ws/src/aws-robomaker-small-warehouse-world/models"\
                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	              jderobot/robotics-academy-ros2:amazon-warehouse-multi-robot /bin/bash
- Copy python planner to container: docker cp exercise/multi-robot.py <container-id>:/opt/warehouse_ws/src/exercise/multi-robot.py
- Inside the container
    - Source workspace and start Nav2: . ./install/setup.sh </br> ros2 launch amazon_robot_bringup  aws_amazon_robot_multiple.py
    - Estimate pose for robots in RViz
    - python3 src/exercise/multi-robot.py

## Running the Plansys planner
- docker cp plansys_multirobot <container>:/opt/warehouse_ws/src/plansys_multirobot.
- Inside container install plansys: . /opt/warehouse_ws/src/plansys_multirobot/plansys.sh
- colcon build --symlink-install
- Bringup Nav2 simulation: ros2 launch amazon_robot_bringup  aws_amazon_robot_multiple.py
- Bringup Plansys experts: ros2 launch plansys_multirobot plansys_multirobot_launch.py
- Run plansys terminal: ros2 run plansys_terminal plansys_terminal
- Copy txt from pddl/problem_single.txt into terminal
- get plan / run