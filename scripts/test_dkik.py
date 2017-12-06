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

def make_init_poses(limb, timestamp=rospy.Time.now()): 
    hdr = Header(stamp=timestamp, frame_id='base')
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


def limb_ik_request(limb, target_pose):
	assert(limb == 'left' or limb == 'right')

	assert(type(target_pose) == dict)
	assert('left' in target_pose) 
	assert('right' in target_pose)

	# ns service name
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	ik_service = rospy.ServiceProxy(ns, SolvePositionIK)
	ik_request = SolvePositionIKRequest()
	ik_request.pose_stamp.append(target_pose[limb])
	try: 
		rospy.wait_for_service(ns, 5.0)
		ik_response = ik_service(ik_request)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s", e)

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                           resp.result_type)

    # Check IK solver answer's validity
    if (resp_seeds[0] != resp.RESULT_INVALID): 
        seed_str = {
            ikreq.SEED_USER: 'User Provided Seed',
            ikreq.SEED_CURRENT: 'Current Joint Angles',
            ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
           }.get(resp_seeds[0], 'None')

		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

    else: 
    	rospy.logerr("Invalid target pose for limb %s", limb)

    return 
