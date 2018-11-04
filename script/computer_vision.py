#! /usr/bin/env /root/proyek_akhir/bin/python3.5

import numpy as np
import cv2
import rospy
import math

from rostu.msg import frontCameraDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('cv_publisher')

pub1 = rospy.Publisher('rostu/frontCam/BallCoor', frontCameraDetection, queue_size=1)
pub2 = rospy.Publisher('rostu/frontCam/ImgArray', Image, queue_size=1)

rate = rospy.Rate(1000000) #1 uS refresh rate
bridge = CvBridge()

cam = cv2.VideoCapture(1)
# cam = cv2.VideoCapture("krsbi.mp4")
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FPS, 60)

file_kalibrasi = ["field_kalibrasi.txt", "line_kalibrasi.txt", "ball_kalibrasi.txt"]
readedFile = []
ff = []
lower = []
upper = []
mask = [None, None, None]

for i in range(len(file_kalibrasi)):
    file = open("/root/catkin_ws/src/rostu/data/" + file_kalibrasi[i], "r")
    readedFile.append(file.read())
    file.close()
    ff.append(["", "", "", "", "", "", "", ""])

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

exposure = int(ff[0][6])
gain = int(ff[0][7])
lastExposure = exposure
lastGain = gain

for i in range(len(file_kalibrasi)):
    lower.append(np.array([int(ff[i][0]), int(ff[i][1]), int(ff[i][2])]))
    upper.append(np.array([int(ff[i][3]), int(ff[i][4]), int(ff[i][5])]))

cam.set(cv2.CAP_PROP_EXPOSURE, exposure / 10100)
cam.set(cv2.CAP_PROP_GAIN, gain / 254)

baalCoor = frontCameraDetection()

while not rospy.is_shutdown():
    ret, img = cam.read()

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

            baalCoor.x = int(x)
            baalCoor.y = int(y)

    img = cv2.resize(img, (480, 360))
    cropped_img = cv2.resize(cropped_img, (480, 360))
    convexHullMask = cv2.resize(convexHullMask, (480, 360))

    cv2.imshow("img", img)
    cv2.imshow("cropped_img", cropped_img)
    cv2.imshow("mask", convexHullMask)

    try :
        image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
    except CvBridgeError as e:
        print(e)

    pub1.publish(baalCoor)
    pub2.publish(image_message)
    rate.sleep()

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
