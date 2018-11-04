#! /usr/bin/env python

import rospy

from rostu.msg import frontCameraDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

ballCoor = [0, 0]

def callback_ballcoor(msg):
    # print(msg.x, msg.y)
    ballCoor[0] = msg.x
    ballCoor[1] = msg.y

range = None
lastRange = None

def callback_qhd_img_depth(msg):
    global range, lastRange

    try:
        depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)

    range = depth_img[ballCoor[1], ballCoor[0]]

    if range == 0:
    	range = lastRange

    lastRange = range
    print(range)

    rate.sleep()

rospy.init_node('kinect_depth')
rate = rospy.Rate(30)  # 1 S refresh rate

sub1 = rospy.Subscriber('/rostu/frontCam/BallCoor', frontCameraDetection, callback_ballcoor)
sub2 = rospy.Subscriber('/kinect2/sd/image_depth_rect', Image, callback_qhd_img_depth)

rospy.spin()
