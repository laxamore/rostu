#! /usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)
    print(cv_image[226,173])

    # print(cv_image.shape)

    cv2.imshow("img", cv_image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        cv2.destroyAllWindows()
        sub.unregister()
        rospy.signal_shutdown("exit")

rospy.init_node('kinect_depth')

sub = rospy.Subscriber('/kinect2/sd/image_depth_rect', Image, callback)

rospy.spin()