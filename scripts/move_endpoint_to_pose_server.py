#!/usr/bin/env python

import rospy 
from baxter_insaros.srv import MoveEndpointToPose
from baxter_insaros import limb_ik_request
from std_msgs.msg import Header
from geometry_msgs.msg import (
	Pose, 
	PoseStamped
)
import baxter_interface as bxi

class MoveEndpointToPoseServer(): 
	def __init__(self, limb): 
		assert(limb == "left" or limb == "right")
		self.limb = limb
		serv = rospy.Service(
					'baxter_insaros/'+ limb + '/MoveEndpointToPose',
					MoveEndpointToPose, 
					self.move_endpoint_to_pose
					)

	def move_endpoint_to_pose(self, req):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		st_pose = PoseStamped(
				header = hdr, 
				pose = req.pose
				)
		limb_joint = limb_ik_request(self.limb, st_pose)




if __name__ == "__main__": 
	rospy.init_node("move_endpoint_to_pose_server")
	left_serv = MoveEndpointToPoseServer("left")
	right_serv = MoveEndpointToPoseServer("right")
	rospy.spin()