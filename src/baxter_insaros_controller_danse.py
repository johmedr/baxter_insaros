import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_insaros as bir

from exceptions import TypeError 


if __name__ == "__main__":
	print "============ Starting dancing setup"
	my_argv=['joint_states:=/robot/joint_states']
	moveit_commander.roscpp_initialize(my_argv)
	rospy.init_node('move_group_python_interface_tutorial',
	                anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	bir_commander = bir.BaxterInsarosCommander()

	display_trajectory_publisher = rospy.Publisher(
	                                    '/move_group/display_planned_path',
	                                    moveit_msgs.msg.DisplayTrajectory)



	poseL_target = geometry_msgs.msg.Pose()

	poseL_target.orientation.x = -0.5
	poseL_target.orientation.y = 0.81
	poseL_target.orientation.z = -0.26
	poseL_target.orientation.w = -0.09

	poseL_target.position.x = 0.57
	poseL_target.position.y = 0.10
	poseL_target.position.z = 0.05


	#print "============ Generating plan R"
	poseR_target = geometry_msgs.msg.Pose()
	poseR_target.orientation.x = 0.61
	poseR_target.orientation.y = 0.74
	poseR_target.orientation.z = 0.26
	poseR_target.orientation.w = 0.02

	poseR_target.position.x = 0.60
	poseR_target.position.y = -0.05
	poseR_target.position.z = 0.05

	bir_commander.set_target_pose(target_pose_left=poseL_target, target_pose_right=poseR_target)
	plan = bir_commander.plan()
	
	print "<<<<<<<<< Type 'quit' (or 'q') for quit or 'go' (or 'g') \
			to execute the plan "
	usr_valid = False

	while not usr_valid: 
		usr = raw_input()
		usr_valid = True
		if usr == "go" or usr == 'g': 
			bir_commander.go(wait=True)

		elif usr == 'quit' or usr == 'q': 
			quit()

		else: 
			usr_valid = False

