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

	poseL_target.orientation.x = 0.140
	poseL_target.orientation.y = -0.980
	poseL_target.orientation.z = -0.013
	poseL_target.orientation.w = -0.027

	poseL_target.position.x = 0.127579481614
	poseL_target.position.y = 0.651981417433
	poseL_target.position.z = -0.1988352386502


	#print "============ Generating plan R"
	poseR_target = geometry_msgs.msg.Pose()
	poseR_target.orientation.w = -0.0034
	poseR_target.orientation.x = 0.5
	poseR_target.orientation.y = 0.86
	poseR_target.orientation.z = -0.0055

	poseR_target.position.x = 0.077
	poseR_target.position.y = - 0.5
	poseR_target.position.z = - 0.2

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

