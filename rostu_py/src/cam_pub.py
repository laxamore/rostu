#! /usr/bin/env /root/proyek_akhir/bin/python3.5

import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('cv_publisher')

pub = rospy.Publisher('rostu/frontCam/ImgArray', Image, queue_size=1)
rate = rospy.Rate(50) #1 uS refresh rate
bridge = CvBridge()

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cam.set(cv2.CAP_PROP_FPS, 30)

while not rospy.is_shutdown():
    ret, img = cam.read()
    # img = cv2.resize(img, (480, 360))

    try :
        image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        pub.publish(image_message)
    except CvBridgeError as e:
        print(e)

    # cv2.imshow("frame", img)
    rate.sleep();

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
