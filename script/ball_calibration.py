#! /usr/bin/env /root/proyek_akhir/bin/python3.5

import numpy as np
import cv2

cam = cv2.VideoCapture(1)

cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def nothing(x):
    pass

file = open("/root/catkin_ws/src/rostu/data/ball_kalibrasi.txt", "r")
readedFile = file.read()
file.close()

it = 0
ff = ["", "", "", "", "", "", "", ""]
for i in range(len(readedFile)):
    if readedFile[i] == ',':
        it = it + 1
        ff[it] = ""
    elif readedFile[i] == ';':
        break
    else:
        ff[it] = ff[it] + readedFile[i]

exposure = int(ff[6])
gain = int(ff[7])
lastExposure = exposure
lastGain = gain

sampling = False

x_start = 0
y_start = 0
x_end = 0
y_end = 0

lower = np.array([int(ff[0]), int(ff[1]), int(ff[2])])
upper = np.array([int(ff[3]), int(ff[4]), int(ff[5])])

refPt = []
getROI = True

getFromTrackbar = True

def leftClick(event, x, y, flags, param):
    global sampling, x_start, y_start, x_end, y_end, getROI

    if event == cv2.EVENT_LBUTTONDOWN:
        x_start, y_start, x_end, y_end = x, y, x, y
        sampling = True
        getROI = False
    elif event == cv2.EVENT_MOUSEMOVE:
        if sampling:
            x_end, y_end = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        x_end, y_end = x, y

        if x_start > x_end:
            x_temp = x_start
            x_start = x_end
            x_end = x_temp

        if y_start > y_end:
            y_temp = y_start
            y_start = y_end
            y_end = y_temp

        if x_start == x_end or y_start == y_end:
            getROI = False
        else:
            getROI = True
        sampling = False


cv2.namedWindow('cam')

cv2.setMouseCallback('cam', leftClick)

cv2.createTrackbar('Exposure', 'cam', exposure, 1500, nothing)
cv2.createTrackbar('Gain', 'cam', gain, 150, nothing)

cv2.createTrackbar('HLow', 'cam', lower[0], 255, nothing)
cv2.createTrackbar('SLow', 'cam', lower[1], 255, nothing)
cv2.createTrackbar('VLow', 'cam', lower[2], 255, nothing)
cv2.createTrackbar('HUp', 'cam', upper[0], 255, nothing)
cv2.createTrackbar('SUp', 'cam', upper[1], 255, nothing)
cv2.createTrackbar('VUp', 'cam', upper[2], 255, nothing)

while True:
    if cv2.getTrackbarPos('Exposure', 'cam') >= 25:
        exposure = cv2.getTrackbarPos('Exposure', 'cam')
        if exposure != lastExposure:
            cam.set(cv2.CAP_PROP_EXPOSURE, exposure / 10100)
        lastExposure = exposure
    if cv2.getTrackbarPos('Gain', 'cam') >= 25:
        gain = cv2.getTrackbarPos('Gain', 'cam')
        if gain != lastGain:
            cam.set(cv2.CAP_PROP_GAIN, gain / 254)
        lastGain = gain

    if not getROI:
        while True:
            if cv2.getTrackbarPos('Exposure', 'cam') >= 25:
                exposure = cv2.getTrackbarPos('Exposure', 'cam')
                if exposure != lastExposure:
                    cam.set(cv2.CAP_PROP_EXPOSURE, exposure / 10100)
                lastExposure = exposure
            if cv2.getTrackbarPos('Gain', 'cam') >= 25:
                gain = cv2.getTrackbarPos('Gain', 'cam')
                if gain != lastGain:
                    cam.set(cv2.CAP_PROP_GAIN, gain / 254)
                lastGain = gain

            ret, img = cam.read()
            # img = cv2.imread("image1.png")

            if not sampling and not getROI:
                cv2.imshow("cam", img)
            elif sampling and not getROI:
                cv2.rectangle(img, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("cam", img)
            elif getROI:
                cv2.rectangle(img, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("cam", img)

                refPt = [(x_start, y_start), (x_end, y_end)]
                roi = img[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
                hsvRoi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                lower = np.array([hsvRoi[:, :, 0].min(), hsvRoi[:, :, 1].min(), hsvRoi[:, :, 2].min()])
                upper = np.array([hsvRoi[:, :, 0].max(), hsvRoi[:, :, 1].max(), hsvRoi[:, :, 2].max()])
                getFromTrackbar = False
                break

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                getROI = True
                getFromTrackbar = False
                break

    if getFromTrackbar:
        lower = np.array([cv2.getTrackbarPos('HLow', 'cam'), cv2.getTrackbarPos('SLow', 'cam'),
                          cv2.getTrackbarPos('VLow', 'cam')])
        upper = np.array([cv2.getTrackbarPos('HUp', 'cam'), cv2.getTrackbarPos('SUp', 'cam'),
                          cv2.getTrackbarPos('VUp', 'cam')])

        cv2.setTrackbarPos('HLow', 'cam', lower[0])
        cv2.setTrackbarPos('SLow', 'cam', lower[1])
        cv2.setTrackbarPos('VLow', 'cam', lower[2])

        cv2.setTrackbarPos('HUp', 'cam', upper[0])
        cv2.setTrackbarPos('SUp', 'cam', upper[1])
        cv2.setTrackbarPos('VUp', 'cam', upper[2])
    else:
        cv2.setTrackbarPos('HLow', 'cam', lower[0])
        cv2.setTrackbarPos('SLow', 'cam', lower[1])
        cv2.setTrackbarPos('VLow', 'cam', lower[2])

        cv2.setTrackbarPos('HUp', 'cam', upper[0])
        cv2.setTrackbarPos('SUp', 'cam', upper[1])
        cv2.setTrackbarPos('VUp', 'cam', upper[2])

        lower = np.array([cv2.getTrackbarPos('HLow', 'cam'), cv2.getTrackbarPos('SLow', 'cam'),
                          cv2.getTrackbarPos('VLow', 'cam')])
        upper = np.array([cv2.getTrackbarPos('HUp', 'cam'), cv2.getTrackbarPos('SUp', 'cam'),
                          cv2.getTrackbarPos('VUp', 'cam')])
        getFromTrackbar = True

    ret, img = cam.read()
    # img = cv2.imread("image1.png")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 0:
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.putText(img, 'center: {}, {}'.format(int(x), int(y)), (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    img = cv2.resize(img, (480, 360))
    mask = cv2.resize(mask, (480, 360))
    cv2.imshow("cam", img)
    cv2.imshow("mask", mask)

    saveFile = str(lower[0]) + "," + str(lower[1]) + "," + str(lower[2]) + "," + \
                    str(upper[0]) + "," + str(upper[1]) + "," + str(upper[2]) + "," + \
                    str(exposure) + "," + str(gain) + ";"
    file = open("/root/catkin_ws/src/rostu/data/ball_kalibrasi.txt", "w")
    file.write(saveFile)
    file.close()

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()