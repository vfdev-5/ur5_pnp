#!/bin/sh

TERM=xterm
CD=$PWD

# Start ROS Core
$TERM -e "source $CD/devel/setup.bash &&  roscore" &

sleep 5

# Bring up robot
$TERM -e "source $CD/devel/setup.bash && roslaunch  ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.34"  &

sleep 5

# Start moveit planning execution 
$TERM -e "source $CD/devel/setup.bash && roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true" &


# Start rviz
$TERM -e "source $CD/devel/setup.bash && roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
