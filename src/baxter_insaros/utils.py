import struct
import rospy

from geometry_msgs.msg import Pose , PoseStamped
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK, 
    SolvePositionIKRequest,
)

def make_poses(leftpose=None, rightpose=None): 
    assert(leftpose is not None or rightpose is not None)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {'left': PoseStamped(
        header=hdr, 
        pose=Pose() if leftpose is None else leftpose
        ), 
             'right': PoseStamped(
        header=hdr, 
        pose=Pose() if rightpose is None else rightpose
        )}
    return poses

def limb_ik_request(limb, target_pose):
    assert(limb == 'left' or limb == 'right')


    if type(target_pose) is dict:
        assert('left' in target_pose) 
        assert('right' in target_pose)
        target_pose = target_pose[limb]

    assert(isinstance(target_pose, PoseStamped))

    # ns service name
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(ns, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(target_pose)
    try: 
        rospy.wait_for_service(ns, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s", e)

    resp_seeds = struct.unpack('<%dB' % len(ik_response.result_type),
                           ik_response.result_type)

    limb_joints = None

    # Check IK solver answer's validity
    if (resp_seeds[0] != ik_response.RESULT_INVALID): 
        seed_str = {
            ik_request.SEED_USER: 'User Provided Seed',
            ik_request.SEED_CURRENT: 'Current Joint Angles',
            ik_request.SEED_NS_MAP: 'Nullspace Setpoints',
           }.get(resp_seeds[0], 'None')

        limb_joints = dict(
                        zip(
                            ik_response.joints[0].name, 
                            ik_response.joints[0].position
                            )
                        )

    else:   
        rospy.logerr("Invalid target pose for limb %s", limb)

    return limb_joints

