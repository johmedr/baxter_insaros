import sys
import copy
import rospy

import moveit_commander
import moveit_msgs.msg

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped

from baxter_insaros import ik_solve

from exceptions import TypeError, UserWarning


class BaxterInsarosCommander:
    def __init__(self):
        """ 
        BaxterInsarosCommander is an interface for MoveIt! MoveGroupCommander, 
        allowing to set a target pose for Baxter. 

        Usage example can be found in the file 'baxter_insaros_controller_danse.py'.
        """
        self._left_target_pose = None
        self._right_target_pose = None
        self._last_group = None
        self._target_poses_affected_to_commander = False
        self._group_commanders = dict()

        for group in ['both_arms', 'right_arm', 'left_arm']:
            self._group_commanders.update(
                {group: moveit_commander.MoveGroupCommander(group)}
            )
            self._group_commanders[group].set_planner_id(
                "RRTConnectkConfigDefault")

    def clear_target_pose(self, side='both_arms'):
        """
        Clear the stored position for the specified side. 
        Args: 
        - side : either 'right_arm', 'left_arm' or 'both_arms'[default]
        """

        assert(side in ['both_arms', 'right_arm', 'left_arm'])

        if side == 'right_arm' or side == 'both_arms':
            self._right_target_pose = None

        if side == 'left_arm' or side == 'both_arms':
            self._right_target_pose = None

        self._target_poses_affected_to_commander = False

    def _affect_target_poses_to_commander(self):
        if self._left_target_pose is None and self._right_target_pose is None:
            raise UserWarning(
                "No target specified for BaxterInsarosCommander - exiting")

        elif not self._target_poses_affected_to_commander:
            joint_target_left = dict()
            joint_target_right = dict()
            joint_target = dict()

            self._last_group = None

            if self._left_target_pose is not None:
                joint_target_left = ik_solve(
                    target_pose_left=self._left_target_pose)
                if bool(joint_target_left):
                    joint_target.update(joint_target_left)
                    self._last_group = "left_arm"

            if self._right_target_pose is not None:
                joint_target_right = ik_solve(
                    target_pose_right=self._right_target_pose)
                if bool(joint_target_right):
                    joint_target.update(joint_target_right)
                    self._last_group = "both_arms" if self._last_group == "left_arm" else "right_arm"
                # NONE non traite
            self._group_commanders[self._last_group].set_joint_value_target(
                joint_target)

            self._target_poses_affected_to_commander = True

    def plan(self):
        """
        Plans a trajectory to the pose specified with the set_target_pose() function.
        Returns: 
                A MoveIt! style planned trajectory
        """
        self._affect_target_poses_to_commander()
        return self._group_commanders[self._last_group].plan()

    def go(self, wait=True):
        """ 
        Move the robot to the pose specified with the set_target_pose() function
        """
        self._affect_target_poses_to_commander()
        return self._group_commanders[self._last_group].go()

    def _set_left_arm_target_pose(self, target_pose):
        assert(isinstance(target_pose, Pose)
               or isinstance(target_pose, PoseStamped))
        self._left_target_pose = target_pose

    def _set_right_arm_target_pose(self, target_pose):
        assert(isinstance(target_pose, Pose)
               or isinstance(target_pose, PoseStamped))
        self._right_target_pose = target_pose

    def set_target_pose(self, target_pose_right=None, target_pose_left=None):
        """ 
        Set a target pose for the left and/or right arm.
        Args: 
                - target_pose_right : a Pose or PoseStamped object 
                - target_pose_left : a Pose or PoseStamped object 
        """
        if target_pose_right is None and target_pose_left is None:
            raise UserWarning(
                "BaxterInsarosCommander's set_target_pose function called without arguments")

        if target_pose_right is not None:
            self._set_right_arm_target_pose(target_pose_right)

        if target_pose_left is not None:
            self._set_left_arm_target_pose(target_pose_left)
