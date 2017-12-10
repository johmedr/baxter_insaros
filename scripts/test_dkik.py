#!/usr/bin/env python
# -*- coding: utf-8 -*-
import struct
import sys
import rospy 

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK, 
    SolvePositionIKRequest,
)

from baxter_insaros import limb_ik_request

def make_init_poses(): 
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }
    return poses 

def main(): 
    rospy.init_node("test_dkik")
    poses = make_init_poses()
    left_joints = limb_ik_request('left', poses)
    right_joints = limb_ik_request('right', poses)

    print("LEFT SIDE : ")
    print(left_joints)
    
    print("RIGHT SIDE : ")
    print(right_joints)

if __name__ == '__main__': 
    sys.exit(main())