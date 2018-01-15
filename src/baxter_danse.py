import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_insaros.utils as utils
from exceptions import TypeError 
from threading import Thread
import time


if __name__ == "__main__":
	print "============ Starting dancing setup"
	my_argv=['joint_states:=/robot/joint_states']
	moveit_commander.roscpp_initialize(my_argv)
	rospy.init_node('move_group_python_interface_tutorial',
	                anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	groupL = moveit_commander.MoveGroupCommander("left_arm")
	groupL.set_planner_id("RRTConnectkConfigDefault");

	groupR = moveit_commander.MoveGroupCommander("right_arm")
	groupR.set_planner_id("RRTConnectkConfigDefault");

	groupBothArms = moveit_commander.MoveGroupCommander("both_arms")
	groupBothArms.set_planner_id("RRTConnectkConfigDefault");

	display_trajectory_publisher = rospy.Publisher(
	                                    '/move_group/display_planned_path',
	                                    moveit_msgs.msg.DisplayTrajectory)

	print "============ Waiting for RVIZ..."
	#rospy.sleep(10)
	print "============ Starting dancing "

	print "============ Reference frame: %s" % groupL.get_planning_frame()
	print "============ Reference frame: %s" % groupL.get_end_effector_link()

	#print "============ Reference frame: %s" % groupR.get_planning_frame()
	#print "============ Reference frame: %s" % groupR.get_end_effector_link()

	print "============ Robot Groups:"
	print robot.get_group_names()

	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"

	print "============ Generating plan L"
	poseL_target = geometry_msgs.msg.Pose()

	poseL_target.orientation.x = 0.140
	poseL_target.orientation.y = -0.980
	poseL_target.orientation.z = -0.013
	poseL_target.orientation.w = -0.027

	poseL_target.position.x = 0.127579481614
	poseL_target.position.y = 0.651981417433
	poseL_target.position.z = -0.1988352386502


	# IK Solve request
	joint_solution_L = utils.limb_ik_request('left', utils.make_poses(leftpose=poseL_target))
	# print joint_solution_L

	# Joint-space planing request
	if joint_solution_L is None: 
		raise TypeError("Joint solution is None.")
	# groupL.set_joint_value_target(joint_solution_L)



	# Working
	#group_variable_values = groupL.get_current_joint_values()
	#print "========= Current joint values : ", group_variable_values
	#group_variable_values[0] = 1.0
	#groupL.set_joint_value_target(group_variable_values)


	#print "============ Generating plan R"
	poseR_target = geometry_msgs.msg.Pose()
	poseR_target.orientation.w = -0.0034
	poseR_target.orientation.x = 0.5
	poseR_target.orientation.y = 0.86
	poseR_target.orientation.z = -0.0055

	poseR_target.position.x = 0.077
	poseR_target.position.y = - 0.5
	poseR_target.position.z = - 0.2
	joint_solution_R = utils.limb_ik_request('right', utils.make_poses(rightpose=poseR_target))


	if joint_solution_R is None: 
		raise TypeError("Joint solution is None.")

	# groupR.set_joint_value_target(joint_solution_R)

	# planL = groupL.plan()
	# planR = groupR.plan()

	joint_solution = dict() 
	joint_solution.update(joint_solution_L)
	joint_solution.update(joint_solution_R)

	print "==== JointSolution : ", joint_solution

	groupBothArms.set_joint_value_target(joint_solution) 
	groupBothArms.plan() 

	print "<<<<<<<<< Type 'quit' (or 'q') for quit, 'go' (or 'g') \
			to execute the plan or 'repeat' (or 'r') to repeat the plan on RVIZ [NOT WORKING]"
	usr_valid = False
	while not usr_valid: 
		usr = raw_input()
		usr_valid = True
		if usr == "go" or usr == 'g': 
			groupBothArms.go(wait=True)

		elif usr == 'repeat' or usr == 'r': 
			
			groupBothArms.set_joint_value_target(joint_solution) 
			groupBothArms.plan()

		elif usr == 'quit' or usr == 'q': 
			quit()

		else: 
			usr_valid = False

