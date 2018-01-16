# import the necessary packages
from __future__ import print_function
import roslib
from collections import deque
import numpy as np
import argparse
import cv2
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

# import packages tuto
import baxter_interface
import baxter_pykdl
from baxter_pykdl import *
import struct
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)
from baxter_core_msgs.srv import (
  SolvePositionIK,
  SolvePositionIKRequest,
)


class image_converter:
  def __init__(self):
    #img = cv2.imread('pink_bottle_cap.jpg') 
    #construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    self.redLower = (0,50,40)
    self.redUpper = (20,255,255)
    self.pts = deque(maxlen=args["buffer"])

    # if a video path was not supplied, grab the reference
    # to the webcam
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
    #self.caminfo_sub = rospy.Subscriber("/cameras/left_hand_camera/camera_info_std",Image,self.callback)

    self.point_pub = rospy.Publisher("/baxter_visual_srv/left_cam/object_center", Point, queue_size=10) 

    self.state = 0
    
  def my_ik(self):
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    limbL = baxter_interface.Limb('left')
    limbR = baxter_interface.Limb('right')
    
    kin = baxter_kinematics('left')
    J = kin.jacobian()
    fpkin1 = kin.forward_position_kinematics()
    Jpi = kin.jacobian_pseudo_inverse()
    #print(dir(Jpi))
    lamda = 0.05
    fpkin_d = np.mat('[9.23007839e-01 1.55353151e-01 -5.63990303e-02   0 0 0]')
    fpkin =np.mat('[fpkin1[0] fpkin1[1] fpkin1[2] 0 0 0]')
    ep = fpkin_d - fpkin
    ep=ep.transpose()
    #print(Jpi)
    #print(ep)
    from numpy import linalg as LA
    print('la norme : ',LA.norm(ep))
    qp = lamda *np.dot(Jpi,ep)
    #print(qp)
    q =limbL.joint_angles()
    #print(q)
    q_d = q.values() + 0.5* qp 
    #q_d = qp
    #L = np.array([],[])
    poseL = {'left_w0': q_d[0,0], 'left_w1': q_d[0,1], 'left_w2': q_d[0,2], 'left_e0': q_d[0,3], 'left_e1': q_d[0,4], 'left_s0': q_d[0,5], 'left_s1': q_d[0,6]}
    
    limbL.move_to_joint_positions(poseL)

  def callback(self,data):
    if self.state==0:
      self.callback_go_to_init_pos()
    else:
      self.callback_vs(data)

  def callback_go_to_init_pos(self):
    self.state = 1 # assume is already in init pos
     
    #self.my_ik()
    # if distance between current EE position
    # and desired EE position < epsilon
    # state should be equal to one.
    
  def callback_vs(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      #while True:
      # grab the current frame
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, self.redLower, self.redUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    cv2.imshow("Mask", mask)


    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and
      # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      print(center) 
      
      center_point = Point(x=center[0], y=center[1], z=0) 
      self.point_pub.publish(center_point); 
      
      # only proceed if the radius meets a minimum size
      if radius > 10:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(frame, (int(x), int(y)), int(radius),
                   (0, 255, 255), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        # End of if on radius
      # End of if condition on len(cts)
    # update the points queue
    self.pts.appendleft(center)
    # loop over the set of tracked points
    for i in xrange(1, len(self.pts)):
      # if either of the tracked points are None, ignore
      # them
      if self.pts[i - 1] is None or self.pts[i] is None:
        continue
        
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
        
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
 
def main():
  print("Start")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  #rs = baxter_interface.RobotEnable(CHECK_VERSION)
  #init_state = rs.state().enabled

  
  print("After image converter")
  def clean_shutdown():
    print("\nExiting example...")
    if not init_state:
      print("Disabling robot...")
      rs.disable()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    #rospy.on_shutdown(clean_shutdow)
    cv2.destroyAllWindows()


if __name__ == "__main__":
  # execute only if run as a script
  main()
 
