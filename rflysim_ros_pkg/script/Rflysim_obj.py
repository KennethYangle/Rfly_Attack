#!/usr/bin/env python
#coding=utf-8

import socket
import cv2
import numpy as np
import struct
import threading
import time
import struct
from geometry_msgs.msg import *
from std_msgs.msg import UInt64
import rospy
from rflysim_ros_pkg.msg import Obj
import math

import sys

def udp_socket():
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def sendUE4Pos(copterID, vehicleType, MotorRPMSMean, PosE, AngEuler, udp, windowID=-1):
    if windowID < 0:
        for i in range(5):
            buf = struct.pack("3i7f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                              PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
            socket = udp_socket()
            # socket.sendto(buf, ('192.168.110.145', 20010+i))
            socket.sendto(buf, (udp, 20010+i))
    else:
        buf = struct.pack("3i7f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
        socket = udp_socket()
        # socket.sendto(buf, ('192.168.110.145', 20010+windowID))
        socket.sendto(buf, (udp, 20010+windowID))

obj_pos = [0, 0, 0]
obj_angle = [0, 0, 0]
obj_type = 0
obj_id = 0
'''
这里进行了结算，mavros到UE4坐标系
mavros          ue4
pos_x          pos_y
pos_y          pos_x
pos_z          - pos_z

sendUE4Pos(CopterID, VehicleType, RotorSpeed, PosM, AngEulerRad, windowsID = 0)  # -8.086
'''

def obj_cb(msg):
    global obj_pos, obj_angle,obj_type, obj_id, param_x, param_y, param_z, param_yaw, param_ip
    obj_id = msg.id
    obj_type = msg.type
    
    obj_pos = [msg.position.y + param_x, msg.position.x + param_y, - msg.position.z + param_z]
    obj_angle = [msg.angule.y, msg.angule.x, - msg.angule.z - param_yaw]
    sendUE4Pos(obj_id, obj_type, 0, obj_pos, obj_angle, param_ip)  # -8.086

if __name__ == '__main__':
    rospy.init_node('obj_control', anonymous=True)
    rospy.Subscriber("ue4_ros/obj", Obj, obj_cb)
    param_x = rospy.get_param("/obj_control/mav_x")
    param_y = rospy.get_param("/obj_control/mav_y")
    param_z = rospy.get_param("/obj_control/mav_z")
    param_yaw = rospy.get_param("/obj_control/mav_yaw")
    param_ip = rospy.get_param("/obj_control/ip")
    rospy.spin()

