import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_insaros.utils as utils

def planif_baxter(pos_x,pos_y,pos_z,or_x, or_y, or_z, or_w, limb) :
    my_argv=['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(my_argv)
    rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    if (limb =="left") :
        group = moveit_commander.MoveGroupCommander("left_arm")
    else :
        group = moveit_commander.MoveGroupCommander("right_arm")
    
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
    
    print "============ Waiting for RVIZ..."
    #rospy.sleep(10)
    print "============ Starting dancing "

    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    print "============ Generating plan "
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z

    pose_target.orientation.x = or_x
    pose_target.orientation.y = or_y
    pose_target.orientation.z = or_z
    pose_target.orientation.w = or_w 
    group.set_planner_id("RRTConnectkConfigDefault");

    # IK Solve request
    if (limb == "left"):
        joint_solution = utils.limb_ik_request('left', utils.make_poses(leftpose=pose_target))
    else :
       joint_solution = utils.limb_ik_request('right', utils.make_poses(rightpose=pose_target))
    
    print joint_solution 

    # Joint-space planing request
    group.set_joint_value_target(joint_solution)
    plan2 = group.plan()
    print plan2

    group.go(wait=True)

print '===== Debut exec'
i = 2
while (i > 0):
    planif_baxter( 0.57,0.61,0.73,0.27,0.25,0.85,-0.34, 'left')
    #planif_baxter( 0.57,-0.18,0.10,-0.14,0.98,-0.01,0.02, 'right')

    planif_baxter( 0.60,0.12,0.64,0.09,-0.49,0.84,-0.2, 'left')
    #planif_baxter( 0.70,-0.14,-0.11,0.730,0.62,0.24,0.12, 'right')
    i = i-1

