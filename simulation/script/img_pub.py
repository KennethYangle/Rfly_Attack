#!/usr/bin/env python3
#coding=utf-8

import numpy as np
import os, json
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from gazebo_msgs.srv import *

cnt = 0
def image_callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global bridge, cnt, params
    cnt += 1

    img_pos = Float32MultiArray()
    #the process of the data
    # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    np_arr = np.fromstring(data.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    cv2.imshow("img", cv_img)

    hue_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    low_range1 = np.array([0, 100, 40])   #[0, 230, 80]
    high_range1 = np.array([10, 255, 210]) # [5, 256, 200]
    th1 = cv2.inRange(hue_image, low_range1, high_range1)
    low_range2 = np.array([170, 100, 40])   #[0, 230, 80]
    high_range2 = np.array([180, 255, 210]) # [5, 256, 200]
    th2 = cv2.inRange(hue_image, low_range2, high_range2)
    th = th1 + th2
    dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    cv2.imshow("hsv", dilated)
    cv2.waitKey(1)
    M = cv2.moments(dilated, binaryImage = True)
    if M["m00"] != 0:
        img_pos.data = [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), np.sqrt(2*M["m00"]), np.sqrt(2*M["m00"]), 0.8]
    else:
        img_pos.data = [-1.0, -1.0, -1.0, -1.0, -1.0]

    # if cnt > params["cam_lose_cnt"]:
    #     img_pos.data = [0, 0, 0, 0, 0]
    imag_pub.publish(img_pos)


if __name__ == '__main__':
    global params
    setting_file = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","settings.json"))
    setting = json.load(setting_file)
    params = setting["Simulation"]

    global imag_pub
    imag_pub = rospy.Publisher("tracker/pos_image", Float32MultiArray, queue_size=10)  # 发送图像位置
    rospy.init_node('iris_fpv_cam', anonymous=True)

    global bridge
    bridge = CvBridge()
    # rospy.Subscriber('/camera/left/compressed', Image, image_callback)
    rospy.Subscriber('/camera/left/compressed', CompressedImage, image_callback)

    rospy.spin()
