#!/usr/bin/env python

import rospy
import actionlib

from baxter_insaros import limb_ik_request, make_interaction_matrix
from baxter_insaros.msg import VisualServoingAction
from sensor_msgs.msg import Image 
import baxter_interface


from control_msgs.msg import (
	FollowJointTrajectoryAction,
	FollowJointTrajectoryGoal
)

import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class VisualServoingServer(object): 

	def __init__(self, limb, resolution=1000):
		assert(limb == "left" or limb == "right")

		self.limb = limb
		self.limb_iface = baxter_interface.Limb(self.limb)

		self.server = actionlib.SimpleActionServer(
			"baxter_insaros/robot/" + self.limb + "/VisualServoing", 
			VisualServoingAction, self.asserv, False)
		self.server.start() 

		self.camera_topic = rospy.Subscriber("/cameras/" + limb + "_hand_camera/image", Image, self.process_img)
		self.joint_control_topic = rospy.Publisher("/robot/limb/" + limb + "/joint_command")

		self.Zc = 10
		self.error = 10 
		self.u = 0
		self.v = 0
		self.resolution = resolution
		self.lbda = 5

		self.robot_model = 

	def process_img(self, Image): 
		self.error = 5
		self.u = 15 
		self.v = 17

	def asserv(self, goal):
		for i in range(self.resolution): 
			# Ls.shape = 2x6
			Ls = make_interaction_matrix(self.u, self.v, self.Zc)
			# Vc.shape = 1x6
			Vc = - self.lbda * np.linalg.pinv(Ls) * self.error
			# Oc.shape = 1x6
			ep_pose = self.limb_iface.endpoint_pose()
			Oc = np_from_ros_pose(ep_pose)







if __name__ == "__main__": 
	rospy.init_node("VisualServoingServer")
	left_serv = VisualServoingServer("left")
	right_serv = VisualServoingServer("right")
	rospy.spin()