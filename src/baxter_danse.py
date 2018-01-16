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
groupR = moveit_commander.MoveGroupCommander("right_arm")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

print "============ Waiting for RVIZ..."
#rospy.sleep(10)
print "============ Starting dancing "

print "============ Reference frame: %s" % groupL.get_planning_frame()
print "============ Reference frame: %s" % groupL.get_end_effector_link()

print "============ Reference frame: %s" % groupR.get_planning_frame()
print "============ Reference frame: %s" % groupR.get_end_effector_link()

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

poseL_target.orientation.x = 0.30
poseL_target.orientation.y = 0.89
poseL_target.orientation.z = -0.08
poseL_target.orientation.w = 0.30

poseL_target.position.x = 0.66
poseL_target.position.y = 0.24
poseL_target.position.z = -0.10
groupL.set_planner_id("RRTConnectkConfigDefault");
# groupL.set_pose_target(poseL_target)
# groupL.set_goal_tolerance(0.01)



# IK Solve request
joint_solution_left = utils.limb_ik_request('left', utils.make_poses(leftpose=poseL_target))
print joint_solution_left

# Joint-space planing request
groupL.set_joint_value_target(joint_solution_left)
plan2 = groupL.plan()
print plan2


# Working
#group_variable_values = groupL.get_current_joint_values()
#print "========= Current joint values : ", group_variable_values
#group_variable_values[0] = 1.0
#groupL.set_joint_value_target(group_variable_values)

print "============ Generating plan R"
poseR_target = geometry_msgs.msg.Pose()
poseR_target.orientation.x = 0.73
poseR_target.orientation.y = 0.62
poseR_target.orientation.z = 0.24
poseR_target.orientation.w = -0.12

poseR_target.position.x = 0.70
poseR_target.position.y = -0.14
poseR_target.position.z = -0.11
groupR.set_planner_id("RRTConnectkConfigDefault");
##groupR.set_pose_target(poseR_target)

# IK Solve request
joint_solution_right = utils.limb_ik_request('right', utils.make_poses(rightpose=poseR_target))
print joint_solution_right

# Joint-space planing request
groupR.set_joint_value_target(joint_solution_right)
plan3 = groupR.plan()
print plan3

print "============ Waiting while RVIZ displays planL (and planR hopefully...)"
#rospy.sleep(5)

#real robot
groupL.go(wait=True)
groupR.go(wait=True)

##### second movement

# left limb
poseL_target.orientation.x = 0.13
poseL_target.orientation.y = 0.93
poseL_target.orientation.z = -0.18
poseL_target.orientation.w = 0.14

poseL_target.position.x = 0.65
poseL_target.position.y = 0.09
poseL_target.position.z = -0.090
groupL.set_planner_id("RRTConnectkConfigDefault");

# IK Solve request
joint_solution_left = utils.limb_ik_request('left', utils.make_poses(leftpose=poseL_target))
print joint_solution_left

# Joint-space planing request
groupL.set_joint_value_target(joint_solution_left)
plan2 = groupL.plan()

print plan2

print "============ Generating plan R"
poseR_target = geometry_msgs.msg.Pose()
poseR_target.orientation.x = 0.68
poseR_target.orientation.y = 0.63
poseR_target.orientation.z = 0.28
poseR_target.orientation.w = -0.2

poseR_target.position.x = 0.68
poseR_target.position.y = 0.3
poseR_target.position.z = -0.11
groupR.set_planner_id("RRTConnectkConfigDefault");
##groupR.set_pose_target(poseR_target)

# IK Solve request
joint_solution_right = utils.limb_ik_request('right', utils.make_poses(rightpose=poseR_target))
print joint_solution_right

# Joint-space planing request
groupR.set_joint_value_target(joint_solution_right)
plan3 = groupR.plan()
print plan3

groupL.go(wait=True)
groupR.go(wait=True)
