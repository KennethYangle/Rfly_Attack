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
from geometry_msgs.msg import TwistStamped, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, RCIn, HomePosition, PositionTarget, AttitudeTarget
from utils_obs import Utils
from rflysim_ros_pkg.msg import Obj


# Simulation of RealFlight
current_state = State()
ch5, ch6, ch7, ch8, ch9, ch11, ch14 = 0, 0, 0, 0, 1, 1, 1
ch20 = 0
is_initialize_mav, is_initialize_vel, is_initialize_rc, is_initialize_img = False, False, False, False

mav_pos = np.array([0, 0, 0], dtype=np.float64)
mav_vel = np.array([0, 0, 0], dtype=np.float64)
mav_yaw = 0
mav_R = np.zeros((3,3))
# 相机系(RDF)转机体系(FLU)
R_cb = np.array([[ 0,  0,  1],\
                 [-1,  0,  0],\
                 [ 0, -1,  0]], dtype=np.float64)
Initial_pos = [0, 0, 0]
pos_i = [0, 0, 0, 0, 0]
pos_i_raw = [0, 0, 0, 0, 0]
pos_i_ekf = [0, 0, 0, 0, 0]
image_failed_cnt = 0
state_name = "InitializeState"
idle_command = TwistStamped()

sphere_pos = np.array([20., 75., 2.], dtype=np.float64)
sphere_vel = np.array([-5., 1., 2.], dtype=np.float64)
sphere_acc = np.array([0., 0., 0.], dtype=np.float64)


def spin():
    rospy.spin()

def state_cb(msg):
    global current_state
    current_state = msg

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, is_initialize_mav, mav_pitch, mav_roll
    is_initialize_mav = True
    mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_pitch = math.asin(2*(q0*q2 - q1*q3))
    mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    # 机体系(FLU)转世界系(ENU)
    mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]], dtype=np.float64)


def mav_vel_cb(msg):
    global mav_vel, is_initialize_vel
    is_initialize_vel = True
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=np.float64)

def rcin_cb(msg):
    global ch5, ch6, ch7, ch8, ch9, ch11, ch14, is_initialize_rc
    is_initialize_rc = True
    last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = ch5, ch6, ch7, ch8, ch9, ch11, ch14
    chs = msg.channels
    ch5 = 2 if chs[4] < 1300 else 1 if chs[4] < 1700 else 0
    ch6 = 2 if chs[5] < 1300 else 1 if chs[5] < 1700 else 0
    ch7 = 0 if chs[6] < 1300 else 1 if chs[6] < 1700 else 2
    ch8 = 0 if chs[7] < 1300 else 1 if chs[7] < 1700 else 2
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
    global is_initialize_img, pos_i_raw, pos_i, image_failed_cnt
    is_initialize_img = True
    # print("msg_data: {}".format(msg.data))
    if msg.data[0] <= 0:
        image_failed_cnt += 1
    else:
        image_failed_cnt = 0
    if image_failed_cnt <= 20 and image_failed_cnt > 0:
        pass
    else:
        pos_i_raw = msg.data
        pos_i = pos_i_raw
    # print("pos_i_raw: {}".format(pos_i_raw))

def pos_image_ekf_cb(msg):
    global pos_i_ekf, pos_i_raw, pos_i
    pos_i_ekf = msg.data
    # If we don't consider the safety of the aircraft when the target is lost, use pos_i_ekf when pos_i_raw[0]<0.
    if abs(pos_i_ekf[0] - pos_i_raw[0]) < 10 and abs(pos_i_ekf[1] - pos_i_raw[1]) < 10:
        pos_i = pos_i_ekf
    else:
        pos_i = pos_i_raw
    # print("pos_i_ekf: {}".format(pos_i_ekf))
    # print("pos_i: {}".format(pos_i))


def sphere_control(dt, is_move=False):
    global sphere_pos, sphere_vel, sphere_acc
    obj_msg = Obj()
    
    obj_msg.id = 100
    obj_msg.type = 152
    sphere_pos = sphere_pos + sphere_vel * dt * is_move
    sphere_vel = sphere_vel + sphere_acc * dt * is_move
    obj_msg.position.x = sphere_pos[0]
    obj_msg.position.y = sphere_pos[1]
    obj_msg.position.z = sphere_pos[2]
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

def moveByBodyRateThrust(publisher, roll_rate, pitch_rate, yaw_rate, thrust):
    '''
    body frame: FLU
    '''
    target_raw_attitude = AttitudeTarget()
    target_raw_attitude.header.stamp = rospy.Time.now()
    target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE

    target_raw_attitude.body_rate.x = roll_rate
    target_raw_attitude.body_rate.y = pitch_rate
    target_raw_attitude.body_rate.z = yaw_rate
    target_raw_attitude.thrust = thrust
    publisher.publish(target_raw_attitude)

if __name__=="__main__":
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
    rospy.Subscriber("tracker/pos_image_ekf", Float32MultiArray, pos_image_ekf_cb)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_acc_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    local_att_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    # print("Publisher and Subscriber Created")

    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # print("Clients Created")
    rate = rospy.Rate(200)#50
    
    # ensure the connection 
    while(not current_state.connected):
        # print("connected: {}".format(current_state.connected))
        rate.sleep()

    for i in range(100):
        local_vel_pub.publish(idle_command)
        rate.sleep()
        
    # switch into offboard
    # print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    time_last = rospy.Time.now().to_sec()

    # start
    cnt = -1
    controller_reset = True
    while not rospy.is_shutdown():
        # print("time: {}".format(rospy.Time.now().to_sec() - last_request.to_sec()))
        cnt += 1
        # sphere_control()
        time_now = rospy.Time.now().to_sec()
        dt = time_now - time_last
        if MODE == "Simulation":
            sphere_control(dt=dt, is_move=(ch8==1))
        time_last = time_now

        if MODE == "Simulation":
            if ch8 == 0:
                if current_state.mode == "OFFBOARD":
                    resp1 = set_mode_client(0, "POSCTL")	# (uint8 base_mode, string custom_mode)
                if cnt % 500 == 0:
                    print("Enter MANUAL mode")
                Initial_pos = mav_pos
                rate.sleep()
                continue
            else:
                if current_state.mode != "OFFBOARD":
                    resp1 = set_mode_client( 0,offb_set_mode.custom_mode )
                    if resp1.mode_sent and cnt % 500 == 0:
                        print("Offboard enabled")
        
        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R, "R_cb": R_cb, 
                    "mav_yaw": mav_yaw, "Initial_pos": Initial_pos,
                    "sphere_pos": sphere_pos, "sphere_vel": sphere_vel, "sphere_acc": sphere_acc}

        # dlt_pos = sphere_pos - mav_pos
        # print("dlt_pos: {}".format(dlt_pos))
        # print("mav_pos: {}".format(mav_pos))
        
        if ch7 >= 1:
            cmd = u.BacksteppingController(pos_info, pos_i, dt, controller_reset)
            controller_reset = False
            moveByBodyRateThrust(local_att_pub, cmd[0], cmd[1], cmd[2], cmd[3])
            # # 识别到图像才进行角速度控制
            # if pos_i[1] > 0:
            #     # moveByBodyRateThrust(local_att_pub, 0., 0., 0., 0.609)
            #     moveByBodyRateThrust(local_att_pub, cmd[0], cmd[1], cmd[2], cmd[3])
            # # # 否则hover
            # else:
            #     local_vel_pub.publish(idle_command)
        else:
            Initial_pos = mav_pos
            controller_reset = True
            local_vel_pub.publish(idle_command)
        obs_pos = sphere_pos
        rate.sleep()
    rospy.spin()