#!/usr/bin/env python

# #main_rounte and #alt_route must be defined
main_route = list() # Initializing main_route as an array
alt_route  = list() # Initializing alt_route as an array

# Define target locations for the robot to get to, and an alternate location incase the first location fails
# NOTE: There must be an `alternate` target for each target
target_1     = {"position":(4.59182405472, -3.24091506004, 0.00), "orientation":(0.0, 0.0, 0.9420323875, -0.335521952934)}
alt_target_1 = {"position":(4.59182405472, -3.24091506004, 0.00), "orientation":(0.0, 0.0, 0.9420323875, -0.335521952934)}

target_2     = {"position":(1.29053378105, 1.01704406738, 0.0), "orientation":(0.0, 0.0, 0.916894700872, 0.399129186496)}
alt_target_2 = {"position":(1.29053378105, 1.01704406738, 0.0), "orientation":(0.0, 0.0, 0.916894700872, 0.399129186496)}

target_3     = {"position":(0.323370456696, -5.42506456375, 0.0), "orientation":(0.0, 0.0, 0.586887005526, 0.809668847582)}
alt_target_3 = {"position":(0.323370456696, -5.42506456375, 0.0), "orientation":(0.0, 0.0, 0.586887005526, 0.809668847582)}

target_4     = {"position":(0.26845061779, 0.0688400268555, 0.0), "orientation":(0.0, 0.0, -0.0157755937785, 0.999875557578)}
alt_target_4 = {"position":(0.26845061779, 0.0688400268555, 0.0), "orientation":(0.0, 0.0, -0.0157755937785, 0.999875557578)}

target_5     = {"position":(0.359970569611, 0.31720161438, 0.0), "orientation":(0.0, 0.0, -0.151776271811, 0.988414874086)}
alt_target_5 = {"position":(0.359970569611, 0.31720161438, 0.0), "orientation":(0.0, 0.0, -0.151776271811, 0.988414874086)}

# Add targets to the list below in the sequence they should be execueted in
main_route = [target_1    , target_2    , target_3    , target_4    , target_5]
alt_route  = [alt_target_1, alt_target_2, alt_target_3, alt_target_4, alt_target_5]











































#******************************** WARNING !!! **************************************#
# **************************** DO NOT EDIT - START ******************************#

from geometry_msgs.msg import Pose

# Check that data is properly defined
if type(main_route) not in [tuple, list]:
	raise ValueError("main_route of type {0} is not properly defined. Please check this and rerun this script".format(type(main_route)))

if type(alt_route) not in [tuple, list]:
	raise ValueError("alt_route of type {0} is not properly defined. Please check this and rerun this script".format(type(alt_route)))

if len(main_route) != len(alt_route):
	raise ValueError("main_route & alt_route must have the same number of target. There must be an alternate target for each target. Please check this and rerun this script")

# This function converts a target to a pose
def _convert_target_to_pose(target):
	pose = Pose()

	pose.position.x = target["position"][0]
	pose.position.y = target["position"][1]
	pose.position.z = target["position"][2]

	pose.orientation.x = target["orientation"][0]
	pose.orientation.y = target["orientation"][1]
	pose.orientation.z = target["orientation"][2]
	pose.orientation.w = target["orientation"][3]

	return pose


# Convert all targets to pose
main_route = tuple( [_convert_target_to_pose(target) for target in main_route] )
alt_route  = tuple( [_convert_target_to_pose(target) for target in alt_route ] )


# ****************************** DO NOT EDIT - END *******************************#
