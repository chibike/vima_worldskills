#!/usr/bin/env python

import math
import time
import rospy
import targets
import actionlib

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

#**********************************************
# TODO: Explain remaps to Marlon
#**********************************************


#*************************** YOU CAN CHANGE THE VALUES TO MATCH YOUR REQUIREMENT *******************************************************


# Define a variable to indicate how long this node should run before quiting
MAX_RUN_TIME = 600 # Seconds

# Define a global variable to define the state of the start button in it's activated state
# (Defualt is set to False # This function thats the #main_rountine when the #start_btn is pressed
# Read button state - meaning that when the cord is pulled the state of the start button would be false)
START_BTN_PRESSED_STATE = False

# Define the maximum_time_interval in which a pose changed is allowed, if a pose change takes longer than this then and error is thrown
# NOTE: The further your poses are from each other the larger this value should be.
ROUNTINE_MAX_DURATION_PER_STEP = 40 # Seconds


#**************************** END OF CHANGE ******************************************************************************

# Define a variable to store the start time of this program in seconds
START_TIME = time.time()

# Define a variable to store the excution state of the main rountine
MAIN_ROUNTINE_ACTIVE = False

# Initialize and name this node
rospy.init_node('vima_task_manager', anonymous=False)

# Create a #SimpleActionClient
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	
# Waiting for the action server to come up
client.wait_for_server()

# This function calls the #main_rountine when the #start_btn is pressed
def start_btn_toggled(data):
	# Read button state
	start_btn_state = data.data

	# Check if main rountine is running
	# If main rountine is not running and the start button is triggered
	# Flag used to prevent this code from running rountines every time a new start_button message is published
	if (not MAIN_ROUNTINE_ACTIVE) and (start_btn_state == START_BTN_PRESSED_STATE):
		start_main_rountine()
	else:
		pass # Do nothing

# This function sends the goal to the move_base server
def send_goal(goal):
	client.send_goal(goal)
	success = client.wait_for_result(rospy.Duration(ROUNTINE_MAX_DURATION_PER_STEP))
	return success

# This function returns the distance between the robot's current pose and a target pose
def get_distance_from_target(target_pose):
	current_pose = get_current_location()

	x1 = current_pose.position.x
	y1 = current_pose.position.y

	x2 = target_pose.position.x
	y2 = target_pose.position.y

	dx = x2 - x1
	dy = y2 - y1
	distance = math.sqrt(pow(dx, 2) + pow(dy, 2))
	return distance

# This function move the robot the new pose
# If this new pose fails then the robot tries the alternate pose
def goto_pose(new_pose=Pose(), alt_pose=Pose()):
	# Create goal object
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	
	# Set target pose and current time
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = new_pose
	
	rospy.loginfo("Task Master:: Going to next pose")
	successful = send_goal(goal)

	if not successful:
		# Set alternate target pose and current time
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = alt_pose
		
		rospy.loginfo("Task Master:: Going to alternate pose")
		return send_goal(goal)

	return successful

def spin():
	# Keep running until roscore shuts down
	while not rospy.is_shutdown():
		if time.time() - START_TIME >= MAX_RUN_TIME:
			rospy.loginfo("Task Master:: Time up")
			break


# This function handles the task/positions for vima to do
def start_main_rountine():
	# Set Active Flag to prevent this code from running this function every time a new start_button message is published
	MAIN_ROUNTINE_ACTIVE = True

#*************************** YOU CAN CHANGE THIS TO YOUR LOGIC *******************************************************

	# Loop through all targets
	for index in range( len(targets.main_route) ):
		# Get next pose from route
		next_pose = targets.main_route[index]
		alt_pose  = targets.alt_route[index]

		# Goto to next_pose, if that fails goto alternate pose
		goto_pose(next_pose, alt_pose)

#**************************** END OF CHANGE ******************************************************************************
	rospy.loginfo("Task Master:: Done")
	spin()


# Subscribe to the start button
rospy.Subscriber('start_btn', Bool, start_btn_toggled)

# Keep running until roscore shuts down
spin()

# Stop running and quit
exit()