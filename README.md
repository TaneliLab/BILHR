## BILHR
code from bio-inspired learning for humanoid robot

## Demo
# Penalty kick
[![Watch the video](https://img.youtube.com/vi/ahFz97uuS7s/hqdefault.jpg)](https://www.youtube.com/watch?v=ahFz97uuS7s)

# CMAC
[![Watch the video](https://img.youtube.com/vi/AfeHUd8koWA/hqdefault.jpg)](https://www.youtube.com/shorts/AfeHUd8koWA)

# For starting Nao:
1. roscore
2. roslaunch nao_bringup nao_full_py.launch
3. roslaunch nao_apps tactile.launch

# Nao stiffness:
1. rosservice call /body_stiffness/enable "{}"
2. rosservice call /body_stiffness/disable "{}"

# ROS things:
1. Creating a new package:  catkin create pkg tutorial_1 --catkin-deps rospy std_msgs
2. Source a new package:    source devel/setup.bash
3. Run a node:              rosrun package-name name-of-python-file
