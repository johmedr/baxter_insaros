import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_insaros.utils as utils

print "============ Starting dancing setup"
my_argv=['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(my_argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

groupL = moveit_commander.MoveGroupCommander("left_arm")
#groupR = moveit_commander.MoveGroupCommander("right_arm")

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
#poseL_target.orientation.x = -0.366894936773
#poseL_target.orientation.y = 0.885980397775
#poseL_target.orientation.z = 0.108155782462
#poseL_target.orientation.w = 0.262162481772

poseL_target.orientation.x = 0.140
poseL_target.orientation.y = -0.989
poseL_target.orientation.z = -0.013
poseL_target.orientation.w = -0.027

poseL_target.position.x = 0.657579481614
poseL_target.position.y = 0.651981417433
poseL_target.position.z = 0.6388352386502
groupL.set_planner_id("RRTConnectkConfigDefault");
# groupL.set_pose_target(poseL_target)
# groupL.set_goal_tolerance(0.01)



# IK Solve request
joint_solution = utils.limb_ik_request('left', utils.make_poses(leftpose=poseL_target))
print joint_solution

# Joint-space planing request
groupL.set_joint_value_target(joint_solution)
plan2 = groupL.plan()
print plan2


# Working
#group_variable_values = groupL.get_current_joint_values()
#print "========= Current joint values : ", group_variable_values
#group_variable_values[0] = 1.0
#groupL.set_joint_value_target(group_variable_values)


#print "============ Generating plan R"
#poseR_target = geometry_msgs.msg.Pose()
#poseR_target.orientation.w = 1.0
#poseR_target.position.x = -0.7
#poseR_target.position.y = 0.05
#poseR_target.position.z = 1.1
#groupR.set_pose_target(poseR_target)

planL = groupL.plan()
#planR = groupR.plan()
print "============ Waiting while RVIZ displays planL (and planR hopefully...)"
#rospy.sleep(5)

#real robot
groupL.go(wait=True)
#groupR.go(wait=True)
