#! /usr/bin/env python

import numpy as np
import cv2
import rospy
import math

from rostu.msg import frontCameraDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('kinect_cv')

pub1 = rospy.Publisher('rostu/frontCam/BallCoor', frontCameraDetection, queue_size=1)

rate = rospy.Rate(30)  # 30 hz refresh rate
bridge = CvBridge()

file_kalibrasi = ["kinect_field_kalibrasi.txt", "kinect_line_kalibrasi.txt", "kinect_ball_kalibrasi.txt"]

readedFile = []
ff = []
lower = []
upper = []
mask = [None, None, None]

for i in range(len(file_kalibrasi)):
    file = open("/root/catkin_ws/src/rostu/data/" + file_kalibrasi[i], "r")
    readedFile.append(file.read())
    file.close()
    ff.append(["", "", "", "", "", ""])

for i in range(len(readedFile)):
    it = 0
    for j in range(len(readedFile[i])):
        if readedFile[i][j] == ',':
            it = it + 1
            ff[i][it] = ""
        elif readedFile[i][j] == ';':
            break
        else:
            ff[i][it] = ff[i][it] + readedFile[i][j]

for i in range(len(file_kalibrasi)):
    lower.append(np.array([int(ff[i][0]), int(ff[i][1]), int(ff[i][2])]))
    upper.append(np.array([int(ff[i][3]), int(ff[i][4]), int(ff[i][5])]))

ballCoor = frontCameraDetection()

def callback_qhd_img_color(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	img = cv2.resize(img, (512, 424))
    except CvBridgeError as e:
        print(e)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((9, 9), np.uint8)
    for i in range(len(file_kalibrasi) - 1):
        mask[i] = cv2.inRange(hsv, lower[i], upper[i])
        mask[i] = cv2.erode(mask[i], None, iterations=2)
        mask[i] = cv2.dilate(mask[i], None, iterations=2)
        # mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_OPEN, kernel)
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_CLOSE, kernel)

    convexHullMask = mask[0] + mask[1]

    allContour = cv2.findContours(convexHullMask.copy(), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None

    hull = []
    for i in range(len(allContour)):
        hull.append(cv2.convexHull(allContour[i], False))

    newHull = []

    for i in range(len(allContour)):
        arrayIndex = 0

        if len(hull) > 1:
            hullSize = np.zeros(len(hull))
            biggestHull = 0
            for j in range(len(hull)):
                maxX = 0
                maxY = 0
                minX = hull[j][0][0][0]
                minY = hull[j][0][0][1]
                for k in range(len(hull[j])):
                    if hull[j][k][0][0] > maxX:
                        maxX = hull[j][k][0][0]
                    if hull[j][k][0][0] < minX:
                        minX = hull[j][k][0][0]

                    if hull[j][k][0][1] > maxY:
                        maxY = hull[j][k][0][1]
                    if hull[j][k][0][1] < minY:
                        minY = hull[j][k][0][1]
                hullSize[j] = math.sqrt(((maxX - minX) ^ 2) + ((maxY - minY) ^ 2))

            for j in range(len(hullSize)):
                if hullSize[j] > biggestHull:
                    biggestHull = hullSize[j]
                    arrayIndex = j

        newHull = [hull[arrayIndex]]

        try:
            cv2.drawContours(img, newHull, i, (255, 0, 0), 2, 8)
        except:
            pass

    mask_for_crop = np.zeros(img.shape, dtype=np.uint8)
    channel_count = img.shape[2]
    ignore_mask_color = (255,) * channel_count
    cv2.fillPoly(mask_for_crop, newHull, ignore_mask_color)
    cropped_img = cv2.bitwise_and(mask_for_crop, img)

    hsv_cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

    mask[2] = cv2.inRange(hsv_cropped_img, lower[2], upper[2])
    mask[2] = cv2.erode(mask[2], None, iterations=2)
    mask[2] = cv2.dilate(mask[2], None, iterations=2)
    # mask[2] = cv2.morphologyEx(mask[2], cv2.MORPH_OPEN, kernel)
    mask[2] = cv2.morphologyEx(mask[2], cv2.MORPH_CLOSE, kernel)

    ballContour = cv2.findContours(mask[2].copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(ballContour) > 0:
        c = max(ballContour, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 0:
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.putText(img, 'center: {}, {}'.format(int(x), int(y)), (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            ballCoor.x = int(x)
            ballCoor.y = int(y)

    # img = cv2.resize(img, (480, 360))
    # cropped_img = cv2.resize(cropped_img, (480, 360))
    # convexHullMask = cv2.resize(convexHullMask, (480, 360))

    cv2.imshow("img", img)
    # cv2.imshow("cropped_img", cropped_img)
    # cv2.imshow("mask", convexHullMask)

    pub1.publish(ballCoor)
    rate.sleep()

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        cv2.destroyAllWindows()
        sub.unregister()
        rospy.signal_shutdown("exit")


sub = rospy.Subscriber('/kinect2/qhd/image_color', Image, callback_qhd_img_color)

rospy.spin()
