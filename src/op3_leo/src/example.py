#!/usr/bin/env python2.7

import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool

ux = 0
uy = 0

ux1 = 0
uy1 = 0

Kpx = 0.01
#Kix = 1.85
Kdx = 0.000

Kpy = 0.01
Kdy = 0.0001
Tm = 0.1 #100ms

t = 0

def imagecallback(img_msg):
    rospy.loginfo(img_msg.header)

    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    img_to_yuv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2YUV)
    img_to_yuv[:,:,0] = cv2.equalizeHist(img_to_yuv[:,:,0])
    eq_img = cv2.cvtColor(img_to_yuv, cv2.COLOR_YUV2RGB)
    blurred = cv2.GaussianBlur(eq_img, (11, 11), 0)
    #hsv_video = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)

    nR = np.matrix(blurred[:,:,0])
    nG = np.matrix(blurred[:,:,1])
    nB = np.matrix(blurred[:,:,2])

    color = cv2.absdiff(nG,nB)

    _, umbral = cv2.threshold(color,75,255,cv2.THRESH_BINARY)

    mask = cv2.erode(umbral, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    x,y,w,h = cv2.boundingRect(opening)

    xball = int(x+(w/2))
    yball = int(y+(h/2))

    cv2.rectangle(cv_img, (x,y), (x+w,y+h), (255,0,0), 8)
    cv2.circle(cv_img, (xball, yball), 5,  (255,255,255), 5)

    area = w*h
    print(area)

    final_img = bridge.cv2_to_imgmsg(cv_img, "rgb8")
    pub.publish(final_img)

    error(xball, yball, area)


def error(x, y, a):
    x_center = 320
    y_center = 240

    errorx = x_center - x
    errory = y_center - y

    errorx = errorx * 70/x_center
    errory = errory * 70/y_center

    point = Point()
    point.x = errorx
    point.y = errory

    error_point = Point()
    error_point.x = errorx
    error_point.y = errory

    error_pub.publish(error_point)

    control(point, a)

def control(point, a):
    global ux
    global uy
    global ux1
    global uy1
    global Kpx
    global Kpy
    global Kdx
    global Kdy
    global Tm
    global t

    errx0 = point.x
    erry0 = point.y

    if errx0 != 70 and erry0 != 70:

        errx1 = 0
        erry1 = 0

        errx2 = 0
        erry2 = 0

        #print(errx0)

        #ux = ux1 + ( Kpx + Kdx/Tm)*errx0 + (-Kpx + Kix*Tm - 2*Kdx/Tm)*errx1 + (Kdx/Tm)*errx2
        ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2
        #uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-Kpy + Kiy*Tm - 2*Kdy/Tm)*erry1 + (Kdy/Tm)*erry2
        uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-2*Kdy/Tm)*erry1 + (-Kpy + Kdy/Tm)*erry2

        if ux >= 70: ux = 70
        elif ux <= -70: ux = 70

        if uy >= -5: uy = -5
        elif uy <= -70: uy = -70

        ux1 = ux
        errx1 = errx0
        errx2 = errx1

        uy1 = uy
        erry2 = erry1
        erry1 = erry0

        #if ux > 3 or ux < -3:

        if errx0 < -5 or errx0 > 5:

            ux = (ux*math.pi)/180
            uy = (uy*math.pi)/180

            position_point = Point()
            position_point.x = ux
            position_point.y = 0
            position_point.z = a

            position_pub.publish(position_point)
    else:
        ux1 = 0
        errx1 = 0
        errx2 = 0

        uy1 = 0
        erry2 = 0
        erry1 = 0
        t += 1

        print(t)
        if (t>360):
            t=-360
        ux_noball = 70*math.sin(0.1*t); #(t/(180/3.14159))

        uy_noball = -10

        ux_noball = (ux_noball*math.pi)/180
        uy_noball = (uy_noball*math.pi)/180

        position_point = Point()

        position_point.x = ux_noball
        position_point.y = uy_noball
        position_point.z = a

        position_pub.publish(position_point)

    #print(ux, uy)

rospy.init_node('image')
pub = rospy.Publisher('/tracking', sensor_msgs.msg.Image, queue_size=1)
error_pub = rospy.Publisher('/error', geometry_msgs.msg.Point, queue_size=1)
position_pub = rospy.Publisher('/position', geometry_msgs.msg.Point, queue_size=1)
mov_pub = rospy.Publisher('/following', std_msgs.msg.Bool, queue_size=1)

rospy.loginfo("Hello ros")

bridge = CvBridge()

subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imagecallback)

kernel = np.ones((5,5),np.uint8)

while not rospy.is_shutdown():
    pass