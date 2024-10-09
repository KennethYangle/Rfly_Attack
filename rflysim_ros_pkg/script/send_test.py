#!/usr/bin/env python3
#coding=utf-8

import socket
import cv2
import numpy as np
import struct
import threading
import time
import struct
# import PX4MavCtrlV4 as PX4MavCtrl
from geometry_msgs.msg import *
from std_msgs.msg import UInt64
import rospy
from rflysim_ros_pkg.msg import Obj

obj_pos = [0, 0, 0]
obj_angle = [0, 0, 0]
obj_type = 0
obj_id = 0
'''
这里进行了结算，mavros到UE4坐标系，有待进一步测试
mavros          ue4
pos_x          pos_y
pos_y          pos_x
pos_z          - pos_z

sendUE4Pos(CopterID, VehicleType, RotorSpeed, PosM, AngEulerRad, windowsID = 0)  # -8.086
msg.twist.linear 代表发送UE4的的位置坐标,即PosM
msg.twist.angular 代表发送到UE4的姿态角，即AngEulerRad
'''
def send_control():
    rospy.init_node('obj_send', anonymous=True)
    obj_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
    
    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate)
    
    obj_msg = Obj()
    obj_msg.id = 100
    obj_msg.type = 152
    obj_msg.position.x = 0
    obj_msg.position.y = 3
    obj_msg.position.z = 2
    obj_msg.angule.x = 0
    obj_msg.angule.y = 0
    obj_msg.angule.z = 0
    obj_msg.size.x = 1
    obj_msg.size.y = 1
    obj_msg.size.z = 1
    while not rospy.is_shutdown():
        obj_msg.position.y = obj_msg.position.y + 0.001
        obj_pub.publish(obj_msg)
        rate.sleep()

if __name__ == '__main__':
    send_control()

