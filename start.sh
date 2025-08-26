#!/bin/bash
set -e

# start.sh - open three GNOME terminals to run ROS processes
# Adjust ROS distro path below if you don't use 'noetic'
if [ -f ~/catkin_ws/devel/setup.bash ]; then
	# shellcheck disable=SC1091
	source ~/catkin_ws/devel/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
	# shellcheck disable=SC1091
	source /opt/ros/noetic/setup.bash
fi

gnome-terminal -- bash -c "roscore; exec bash"
sleep 10
gnome-terminal -- bash -c "roslaunch dr200_description run.launch odom_publish_tf:=false; exec bash"
sleep 10
gnome-terminal -- bash -c "roslaunch bot_navigation navigation.launch use_ekf:=true open_rviz:=true slow_mode:=true move_forward_only:=false; exec bash"

exit 0

