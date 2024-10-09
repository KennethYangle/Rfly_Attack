#!/usr/bin/env python3
#coding=utf-8

import rospy
import os
import json
import numpy as np
import math
import time
import threading
import tkinter
from geometry_msgs.msg import *
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandBool, CommandTOL
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetMavFrame
from mavros_msgs.msg import State, RCIn, HomePosition
from mavros_msgs.msg import Thrust
from utils_obs import Utils
from Queue import Queue
from rflysim_ros_pkg.msg import Obj
from R_Client import RClient
import pickle

np.random.seed(777)

# Simulation of RealFlight
current_state = State()
ch5, ch6, ch7, ch8, ch9, ch11, ch14 = 0, 0, 0, 0, 1, 1, 1
is_initialize_mav, is_initialize_vel, is_initialize_rc, is_initialize_img = False, False, False, False

ch20 = 0
mav_pos = [0, 0, 0]
mav_original_angle = [0, 0, 0]
# mav_vel = [0, 0, 0]
mav_vel = np.array([0, 0, 0])
mav_yaw = 0
mav_R = np.zeros((3,3))
Initial_pos = [0, 0, 0]
pos_i = [0, 0, 0, 0, 0]
image_failed_cnt = 0
state_name = "InitializeState"
command = TwistStamped()
takeoff_command = TwistStamped()
q = Queue()
maxQ = 100
sumQ = 0.0
home_dx, home_dy = 0, 0
depth = -1
original_offset = np.array([0, 0, 0])

sphere_pos_x, sphere_pos_y, sphere_pos_z = 0, 18, 3  #-0.065, 7, 2.43 
vec = np.random.randn(3)
vec /= np.linalg.norm(vec)
v_x, v_y, v_z = vec[0], vec[1], vec[2]
sphere_vx, sphere_vy, sphere_vz = -1, 0, 0

sphere_feb_pos = PoseStamped()
# obj_state = ModelState()

datas = []

def spin():
    rospy.spin()

def state_cb(msg):
    global current_state
    current_state = msg

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, is_initialize_mav, mav_pitch, mav_roll
    is_initialize_mav = True
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_pitch = math.asin(2*(q0*q2 - q1*q3))
    mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
    # R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to body_coordinate
    R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to baselink_coordinate  // body to enu  # body: right-front-up3rpos_est_body)
    mav_R = R_ae.dot(R_ba)
    # mav_R = R_ae

def mav_vel_cb(msg):
    global mav_vel, is_initialize_vel
    is_initialize_vel = True
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

def rcin_cb(msg):
    global ch5, ch6, ch7, ch8, ch9, ch11, ch14, is_initialize_rc
    is_initialize_rc = True
    last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = ch5, ch6, ch7, ch8, ch9, ch11, ch14
    chs = msg.channels
    ch5 = 2 if chs[4] < 1300 else 1 if chs[4] < 1700 else 0
    ch6 = 2 if chs[5] < 1300 else 1 if chs[5] < 1700 else 0
    ch7 = 2 if chs[6] < 1300 else 1 if chs[6] < 1700 else 0
    ch8 = 2 if chs[7] < 1300 else 1 if chs[7] < 1700 else 0
    ch9 = 2 if chs[8] < 1300 else 1 if chs[8] < 1700 else 0
    ch11 = 1 if chs[10] < 1500 else 0
    ch14 = 1 if chs[10] < 1500 else 0
    if ch5!=last_ch5 or ch6!=last_ch6 or ch7!=last_ch7 or ch8!=last_ch8 or ch9!=last_ch9 or ch11!=last_ch11 or ch14!=last_ch14:
        print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(ch5, ch6, ch7, ch8, ch9, ch11, ch14))

def call(event):
    global ch5, ch6, ch7, ch8, ch9, ch20
    k = event.keysym
    if k == "m":
        ch6 = 0
    elif k == "h":
        ch20 = 1
    elif k == "o":
        ch8 = 1
    elif k == "p":
        ch8 = 0
    elif k == "c":
        ch9 = (ch9 + 1) % 2
    elif k == "a":
        ch7 = 1
    elif k == "b":
        ch7 = 0
    time.sleep(0.02)

def read_kbd_input():
    global is_initialize_rc
    is_initialize_rc = True
    win = tkinter.Tk()
    frame = tkinter.Frame(win,width=100,height=60)
    frame.bind("<Key>",call)
    frame.focus_set()
    frame.pack()
    win.mainloop()

def pos_image_cb(msg):
    global is_initialize_img, pos_i, image_failed_cnt
    is_initialize_img = True
    # # print("msg_data: {}".format(msg.data))
    if msg.data[0] <= 0:
        image_failed_cnt += 1
    else:
        image_failed_cnt = 0
    if image_failed_cnt <= 20 and image_failed_cnt > 0:
        pass
    else:
        pos_i = msg.data
    # # print("pos_i: {}".format(pos_i))

def setArm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
       armService(True)
       # print("Arming!")
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e

def setDisarm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
       armService(False)
       # print("Disarming!")
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e

def setTakeoff(offb_set_mode):
    diff_pos = np.array([mav_pos[0], mav_pos[1], 5.0]) - np.array(mav_pos)
    # for i in range(150):
    while np.linalg.norm(diff_pos) > 0.1:
        if not current_state.armed: setArm()
        if current_state.mode != "OFFBOARD": set_mode_client( 0,offb_set_mode.custom_mode )
        tkcmd = diff_pos
        takeoff_command.twist.linear.x = tkcmd[0]
        takeoff_command.twist.linear.y = tkcmd[1]
        takeoff_command.twist.linear.z = tkcmd[2]
        takeoff_command.twist.angular.z = 0.
        local_vel_pub.publish(takeoff_command)
        time.sleep(0.02)
        diff_pos = np.array([mav_pos[0], mav_pos[1], 5.0]) - np.array(mav_pos)

def setLandMode():
   rospy.wait_for_service('/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e


def sphere_control(cnt):
    global sphere_pos_x, sphere_pos_y, sphere_pos_z, v_x, v_y, v_z
    obj_msg = Obj()
    
    obj_msg.id = 100
    obj_msg.type = 152
    # sphere_pos_x = 3*np.sin(cnt*1.0/500)
    sphere_pos_x += v_x * 0.01
    sphere_pos_y += v_y * 0.01
    sphere_pos_z += v_z * 0.01
    obj_msg.position.x = sphere_pos_x
    obj_msg.position.y = sphere_pos_y
    obj_msg.position.z = sphere_pos_z
    obj_msg.size.x = 0.2
    obj_msg.size.y = 0.2
    obj_msg.size.z = 0.2

    sphere_pub.publish(obj_msg)

def minAngleDiff(a, b):
    diff = a - b
    if diff < 0:
        diff += 2*np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2*np.pi

def angleLimiting(a):
    if a > np.pi:
        return a - 2*np.pi
    if a < -np.pi:
        return a + 2*np.pi
    return a



if __name__=="__main__":
    r = RClient()
    # Start RflySim
    r.Start()
    time.sleep(40)

    setting_file = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","settings.json"))
    setting = json.load(setting_file)
    # print(json.dumps(setting, indent=4))

    MODE = setting["MODE"]
    car_velocity = setting["car_velocity"]
    # follow_mode: 0, ll=follow_distance; 1, ll=norm(car_home, mav_home)
    follow_mode = setting["follow_mode"]
    follow_distance = setting["follow_distance"]
    FLIGHT_H = setting["FLIGHT_H"]
    if MODE == "RealFlight":
        u = Utils(setting["Utils"])
    elif MODE == "Simulation":
        u = Utils(setting["Simulation"])

    rospy.init_node('offb_node', anonymous=True)
    spin_thread = threading.Thread(target = spin)
    spin_thread.start()

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    #HIL使用遥控器进行控制
    IsRC = setting["IsRC"]
    if MODE == "RealFlight":
        rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        image_center = [setting["Utils"]["WIDTH"] / 2.0, setting["Utils"]["HEIGHT"] / 2.0]
    elif MODE == "Simulation":
        sphere_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
        image_center = [setting["Simulation"]["WIDTH"] / 2.0, setting["Simulation"]["HEIGHT"] / 2.0]

        if IsRC == True:
            rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        else:
            inputThread = threading.Thread(target=read_kbd_input)
            inputThread.start()
    else:
        raise Exception("Invalid MODE!", MODE)
    rospy.Subscriber("tracker/pos_image", Float32MultiArray, pos_image_cb)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # print("Publisher and Subscriber Created")

    # rospy.wait_for_service("mavros/setpoint_velocity/mav_frame")
    # frame_client = rospy.ServiceProxy('mavros/setpoint_velocity/mav_frame', SetMavFrame)
    # resp_frame = frame_client(8)
    # if resp_frame.success:
    #     # print("Set earth_FLU success!")
    # else:
    #     # print("Set frame failed!")

    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # print("Clients Created")
    rate = rospy.Rate(50)#50
    
    # ensure the connection 
    while(not current_state.connected):
        # print("connected: {}".format(current_state.connected))
        rate.sleep()

    for i in range(100):
        local_vel_pub.publish(command)
        rate.sleep()
        
    # switch into offboard
    # print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"

    last_request = rospy.Time.now()

    # start
    statistic_state = "start"
    episode = 0
    dlt_pos_stash = list()
    cnt = -1
    controller_reset = True
    data = {"sphere_traj": [], "mav_traj": []}
    while not rospy.is_shutdown():
        # print("time: {}".format(rospy.Time.now().to_sec() - last_request.to_sec()))
        # print("episode: {}".format(episode))
        cnt += 1

        if statistic_state == "start":
            if episode > 0:
                r.Start()
                time.sleep(40)

            sphere_control(cnt)
            time.sleep(0.2)
            setArm()
            # Enter Offboard mode
            if current_state.mode != "OFFBOARD":
                resp1 = set_mode_client( 0,offb_set_mode.custom_mode )
                if resp1.mode_sent:
                    print("Offboard enabled")
                last_request = rospy.Time.now()
            setTakeoff(offb_set_mode)
            statistic_state = "mission"
            obs_pos = [sphere_pos_x, sphere_pos_y, sphere_pos_z]
            data["sphere_traj"].append(obs_pos)
            data["mav_traj"].append(mav_pos)

        if statistic_state == "mission":
            sphere_control(cnt)
            pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R, "R_bc": np.array([[0,0,1], [1,0,0], [0,1,0]]), 
                        "mav_original_angle": mav_original_angle, "Initial_pos": Initial_pos}

            dlt_pos = np.array([sphere_pos_x, sphere_pos_y, sphere_pos_z]) - np.array(mav_pos)
            # print("dlt_pos: {}".format(dlt_pos))
            dlt_pos_stash.append(np.linalg.norm(dlt_pos))

            cmd = u.RotateAttackController(pos_info, pos_i, image_center, controller_reset)
            # cmd = u.BasicAttackController(pos_info, pos_i, image_center)
            # 识别到图像才进行角速度控制
            if pos_i[1] > 0: 
                command.twist.linear.x = cmd[0]
                command.twist.linear.y = cmd[1]
                command.twist.linear.z = cmd[2]
                command.twist.angular.z = cmd[3]
                # print("cmd: {}".format(cmd))
            # # 否则hover
            else:
                command.twist.linear.x = 0.
                command.twist.linear.y = 0.
                command.twist.linear.z = 0.
                command.twist.angular.z = 0.
                statistic_state = "stop"

            obs_pos = [sphere_pos_x, sphere_pos_y, sphere_pos_z]
            data["sphere_traj"].append(obs_pos)
            data["mav_traj"].append(mav_pos)
            controller_reset = False

            # print("command: {}".format([command.twist.linear.x, command.twist.linear.y, command.twist.linear.z, command.twist.angular.z]))
            local_vel_pub.publish(command)
            rate.sleep()

        if statistic_state == "stop":
            r.Stop()
            time.sleep(1)
            if current_state.mode == "OFFBOARD":
                resp1 = set_mode_client(0, "POSCTL")	# (uint8 base_mode, string custom_mode)
                # print("Enter MANUAL mode")

            min_distance = min(dlt_pos_stash)
            # print("dlt_pos_stash: {}".format(dlt_pos_stash))
            # print("min_distance: {}".format(min_distance))
            data["min_distance"] = min_distance
            datas.append(data)
            f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","datas.pkl"), 'w')
            pickle.dump(datas, f)
            f.close()

            dlt_pos_stash = list()
            sphere_pos_x, sphere_pos_y, sphere_pos_z = 0, 18, 3  #-0.065, 7, 2.43 
            vec = np.random.randn(3)
            vec /= np.linalg.norm(vec)
            vec *= 0
            v_x, v_y, v_z = vec[0], vec[1], vec[2]
            cnt = -1
            controller_reset = True
            data = {"sphere_traj": [], "mav_traj": []}
            statistic_state = "start"
            episode += 1
            if episode > 60:
                break
