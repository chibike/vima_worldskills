#!/bin/bash

MAP_FILE_NAME=/home/chibike/my_demo_map
SLEEP_TIME=3
MAP_UPDATE_INTERVAL=300

# ROS Parameters
ANGULAR_UPDATE=0.1 # How far the robot has to rotate before a new scan is considered for inclusion in the map
LINEAR_UPDATE=0.1 # How far the robot has to move before a new scan is considered for inclusion in the map
LSKIP=10 # How many beams to skip when processing each LaserScan message

echo "Robot is running gmapping<y/N>?"
read x
if [ $x = "y" ]
then
	echo ""
else
	echo "Exiting...."
	exit
fi

echo "Enter map name (e.g test_map)?"
read MAP_FILE_NAME

echo "Do you want to record map data (y/N)"
read x
if [ $x = "y" ]
then
    echo "Enter rosbag file name (e.g data.bag)?"
    read ROSBAG_FILE_NAME
    echo "Running rosbag"
    gnome-terminal --command="rosbag record -O $ROSBAG_FILE_NAME /scan /tf"
fi

echo "Running rviz"
gnome-terminal --command="roslaunch turtlebot_rviz_launchers view_navigation.launch"

echo "Running keyboard teleop"
gnome-terminal --command="roslaunch turtlebot_teleop keyboard_teleop.launch"

while :
do
    sleep $MAP_UPDATE_INTERVAL
    rosrun map_server map_saver -f $MAP_FILE_NAME
done