import struct
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def ik_solve(target_pose=None, limb='both', target_pose_right=None, target_pose_left=None):
    """ 
    Uses Baxter core IK solver to compute joint positions leading to target_pose. 

    Usages:

    1. ik_solve(limb, target_pose) 
            - limb : can be either 'left', 'right' or 'both'[default]
            - target_pose : a Pose or PoseStamped object if limb is 'left' or 'right',  
              a dict containing the keys 'left' and 'right' (with value formatted as Pose 
              or PoseStamped) if limb is 'both'

    2. ik_solve(target_pose_right, target_pose_left)[prefered]
            - target_pose_right : a Pose or PoseStamped object 
            - target_pose_left : idem

    returns :
            - joint positions as a dictionnary of {'joint': 'value'} if found, else None 

    NB: if IK is request for both arms and a solution could not be found on one, 
    the joint positions dictionnary will not contain information for each joint. 
    Using it with a 'both_arms' move group of MoveIt! could lead to an unexpected 
    behavior. 
    """

    _target_pose = {'left': None, 'right': None}

    if target_pose is not None:
        assert(limb == 'left' or limb == 'right' or limb == 'both')
        assert(target_pose_left is None and target_pose_right is None)

        if limb == 'left' or limb == 'right':
            _target_pose[limb] = target_pose

        elif limb == 'both':
            assert(isinstance(target_pose, dict))
            assert('left' in target_pose)
            assert('right' in target_pose)

            for l in ['left', 'right']:
                _target_pose[l] = target_pose
    else:
        assert(target_pose_right is not None or target_pose_left is not None)

        if target_pose_right is not None:
            _target_pose['right'] = target_pose_right

        if target_pose_left is not None:
            _target_pose['left'] = target_pose_left

    joints = dict()

    for l, pose in _target_pose.iteritems():
        if pose is None:
            continue
        else:
            fetched_joints = _ik_solve_one_limb(l, pose)

            if fetched_joints is not None:
                joints.update(fetched_joints)

    return joints


def _ik_solve_one_limb(limb, target_pose):
    assert(limb == 'left' or limb == 'right')

    # if type(target_pose) is dict:
    #     assert('left' in target_pose)
    #     assert('right' in target_pose)
    #     target_pose = target_pose[limb]
    if(isinstance(target_pose, Pose)):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        target_pose = PoseStamped(header=hdr, pose=target_pose)
    else:
        assert(isinstance(target_pose, PoseStamped))

    ik_response, ik_request = _ik_send_srv_request(limb, target_pose)

    limb_joints = None

    if ik_response is not None:
        limb_joints = _ik_parse_srv_response(ik_response, ik_request)

    return limb_joints


def _ik_send_srv_request(limb, target_pose):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(ns, SolvePositionIK)

    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(target_pose)

    ik_response = None

    try:
        rospy.wait_for_service(ns, 5.0)
        ik_response = ik_service(ik_request)

    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("BaxterInsarosController : Service call failed: %s", e)

    return ik_response, ik_request


def _ik_parse_srv_response(ik_response, ik_request):
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
        rospy.logerr("BaxterInsarosController : Invalid target pose")

    return limb_joints
