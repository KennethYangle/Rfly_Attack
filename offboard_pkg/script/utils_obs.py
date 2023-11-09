#!/usr/bin/env python
#coding=utf-8

import numpy as np
import math
import tf

# 定义点的函数
class Point():
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def getx(self):
        return self.x

    def gety(self):
        return self.y

# 定义直线函数
class Getlen():
    def __init__(self, p1, p2):
        self.x = p1.getx() - p2.getx()
        self.y = p1.gety() - p2.gety()
        # 用math.sqrt（）求平方根
        self.len = math.sqrt((self.x ** 2) + (self.y ** 2))

    # 定义得到直线长度的函数
    def getlen(self):
        return self.len


def vex(matrix_A):
    return np.array([matrix_A[2, 1], matrix_A[0, 2], matrix_A[1, 0]])


def matrix(B, C):
    return np.array([[B[0] * C[0], B[0] * C[1], B[0] * C[2]],
                     [B[1] * C[0], B[1] * C[1], B[1] * C[2]],
                     [B[2] * C[0], B[2] * C[1], B[2] * C[2]]])


class Utils(object):
    def __init__(self, params):
        self.WIDTH = params["WIDTH"] #simulation 720  Real_flight:640
        self.HEIGHT = params["HEIGHT"] #simulation 405  Real_flight:405

        self.circlex = None
        self.circley = None
        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2.
        self.v0 = self.h*0.42 # self.h*0.43 # self.h/2
        self.x0 = self.u0
        self.y0 = self.v0
        self.cnt = 0
        self.cnt_WP = 1
        self.v_norm_d = 10
        self.last_mav_vel = np.array([[0], [0], [0]], dtype=np.float64)
        #realsense: fx:632.9640658678117  fy:638.2668942402212
        self.f = params["FOC"] #150 #346.6  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改
        #camrea frame to mavros_body frame
        self.R_cb = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.R_origin = np.array([[0,0,1],\
                             [1,0,0],\
                             [0,1,0]])
        self.n_cc = np.array([0,0,1])

        self.vd = np.array([0, 0, 0], dtype=np.float64)
        self.ko = 2.5 #the angular rate to avoid obstacle

        self.kp_vd = 2.0 #the p_control about desire velicoty

        self.kvod = 2.5 #the p_control about desire velicoty by matrix initial value:1   2.5


    def distance(self, circle):
        p1 = Point(circle[0], circle[1])
        p2 = Point(self.w / 2, self.h / 2)
        l = Getlen(p1, p2)
        # # print(self.circley)
        return l.getlen()

    def distance_y(self, circle):
        len_y = circle[1] - self.h / 2
        return len_y

    def par(self, d, d1, d2):
        if d < d1 or d == d1:
            return 1.0
        elif (d1 < d and d < d2) or d == d2:
            A = -2 / math.pow((d1 - d2), 3)
            B = 3 * (d1 + d2) / math.pow((d1 - d2), 3)
            C = -6 * d1 * d2 / math.pow((d1 - d2), 3)
            D = (math.pow(d2, 2) * (3 * d1 - d2)) / math.pow((d1 - d2), 3)
            return A * math.pow(d, 3) + B * math.pow(d, 2) + C * math.pow(d, 1) + D
        else:
            return 0.0

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a

    def DockingControllerFusion(self, pos_info, pos_i):
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        self.change = self.par(n_eo.dot(n_ec), 0.867, 0.966)  #0.707, 0.867 the parameter about smooth
        if pos_i[1] == 0:
            self.change = 1
        
        #calculate vod
        vd1 = matrix(n_eo, n_eo)
        matrix_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        vI = matrix_I - vd1
        vod = (1./(1.001 - n_eo.dot(n_ec)))*(1 - self.change) * self.kvod * vI.T.dot(n_ec)
        
        v_rd_b = np.array([0, 0, 0])#np.array([1, 0, 0])
        v_rd_e = pos_info["mav_R"].dot(v_rd_b)
        # print("vr:{}".format(v_rd_e))
        
        v = vod + self.change * v_rd_e
        
        v[0] = v[0]
        v[1] = v[1]
        v[2] = v[2]
        v = self.sat(v,1.0)
        
        # print("v:{}".format(v))
        return [v[0], v[1], v[2], 0]

    def BasicAttackController(self, pos_info, pos_i, image_center):
        yaw = pos_info["mav_original_angle"][0]
        cmd = [5*np.cos(yaw), 5*np.sin(yaw), 0.01*(image_center[1] - pos_i[1]), 0.01*(image_center[0] - pos_i[0])]
        # print("pos_i: {}\nimage_center: {}\ncmd: {}".format(pos_i, image_center, cmd))
        return cmd

    def RotateAttackController(self, pos_info, pos_i, image_center, controller_reset):
        if controller_reset: self.cnt = 0
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc
        
        self.cnt += 1
        # v_m[1] = v_b[1] * 0.1/(1.01-cos_beta) + self.sat(self.cnt * 0.1,10)
        v_m = np.array([0., 0., 0.])
        # case1: (0.02, 3, 10)
        # case2: (0.04, 3, 12)
        v_m[1] = self.sat(self.cnt * 0.03, 10)
        v_m[0] = 12*v_b[0]
        v_m[2] = 10*v_b[2]
        # v_f = self.sat(self.cnt*0.02*np.array([0.,1.,0.]), 10)
        # v_m = (1-cos_beta)*v_b + (cos_beta)*v_f
        v = pos_info["mav_R"].dot(v_m)
        # v = self.sat(v, 8)
        yaw_rate = 0.002*(image_center[0] - pos_i[0])
        
        # print("v_b: {}\nv_m: {}\nv: {}".format(v_b, v_m, v))
        # print("yaw_rate: {}".format(yaw_rate))
        return [v[0], v[1], v[2], yaw_rate]

    def RotateAttackAccelerationController(self, pos_info, pos_i, controller_reset):
        if controller_reset: self.cnt = 0
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        # 两种用法：1）给定世界系下固定的n_td，限定打击方向；2）相对光轴一向量，随相机运动
        # n_td = np.array([0, -1, 0], dtype=np.float64)
        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = 2.0 * (n_eo - n_td)   # n_t -> n_td
        v_2 = 1.0 * n_td            # v   -> n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        # v_d *= min(V + 2.5, 12)
        v_d *= V + 2

        a_d = self.sat(1.0 * (v_d - pos_info["mav_vel"]), 6.)

        yaw_rate = 0.002*(self.u0 - pos_i[0])
        
        # print("n_co:{}, n_bo:{}, n_eo:{}, v_1:{}, v_2:{}, v_d:{}".format(n_co, n_bo, n_eo, v_1, v_2, v_d))
        return [a_d[0], a_d[1], a_d[2], yaw_rate]

    def RotateAttackAccelerationController2(self, pos_info, pos_i, controller_reset, yaw_d=np.pi/2):
        if controller_reset: self.cnt = 0
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        # 两种用法：1）给定世界系下固定的n_td，限定打击方向；2）相对光轴一向量，随相机运动
        # n_td = np.array([np.cos(yaw_d), np.sin(yaw_d), 0], dtype=np.float64)
        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = max(2.5 - pos_i[2]/120., 0.5) * (n_eo - n_td)   # n_t -> n_td
        v_2 = 1.0 * n_td            # v   -> n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 1.5
        # v_d *= V + 2.0

        a_d = self.sat(1.0 * (v_d - pos_info["mav_vel"]), 6.)
        a_d[2] = self.sat(a_d[2], 2.)

        yaw_rate = 0.0025*(self.u0 - pos_i[0])
        
        # print("n_co:{}, n_bo:{}, n_eo:{}, v_1:{}, v_2:{}, v_d:{}".format(n_co, n_bo, n_eo, v_1, v_2, v_d))
        return [a_d[0], a_d[1], a_d[2], yaw_rate]

    def RotateAttackAccelerationController2VelCmd(self, pos_info, pos_i, controller_reset, yaw_d=np.pi/2):
        if controller_reset: self.cnt = 0
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        # 两种用法：1）给定世界系下固定的n_td，限定打击方向；2）相对光轴一向量，随相机运动
        # n_td = np.array([np.cos(yaw_d), np.sin(yaw_d), 0], dtype=np.float64)
        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        # n_td = n_ec
        # n_td /= np.linalg.norm(n_td)
        v_1 = max(2.0 - pos_i[2]/100., 0.5) * (n_eo - n_td)   # n_t -> n_td
        v_2 = 1.0 * n_td            # v   -> n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 0.6
        v_d[2] = self.SaftyZ(v_d[2], 1.)
        # v_d *= V + 2.0

        yaw_rate = 0.0025*(self.u0 - pos_i[0])
        
        # print("n_co:{}, n_bo:{}, n_eo:{}, v_1:{}, v_2:{}, v_d:{}".format(n_co, n_bo, n_eo, v_1, v_2, v_d))
        return [v_d[0], v_d[1], v_d[2], yaw_rate]

    def BacksteppingController(self, pos_info, pos_i, dt, controller_reset):
        # params
        m = 1.4
        g = np.array([[0], [0], [-9.801]], dtype=np.float64)
        C_d = 0.073
        thrust_hover = 0.609

        # interceptor and target state
        mav_pos = pos_info["mav_pos"].reshape(-1, 1)    # 将数组转化为列向量
        mav_vel = pos_info["mav_vel"].reshape(-1, 1)
        mav_acc = (mav_vel - self.last_mav_vel) / dt
        self.last_mav_vel = mav_vel
        sphere_pos = pos_info["sphere_pos"].reshape(-1, 1)
        sphere_vel = pos_info["sphere_vel"].reshape(-1, 1)
        sphere_acc = pos_info["sphere_acc"].reshape(-1, 1)

        # calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_c_in_c = self.n_cc.reshape(-1, 1)
        n_c_in_b = pos_info["R_cb"].dot(n_c_in_c)
        n_c_in_e = pos_info["mav_R"].dot(n_c_in_b)
        
        # calacute the no
        n_t_in_c = np.array([[pos_i[0] - self.u0], [pos_i[1] - self.v0], [self.f]], dtype=np.float64)
        n_t_in_c /= np.linalg.norm(n_t_in_c)
        n_t_in_b = pos_info["R_cb"].dot(n_t_in_c)
        n_t_in_e = pos_info["mav_R"].dot(n_t_in_b)

        # n_td design
        n_td_in_e = n_c_in_e

        # L_1
        k_b = 0.25   # 41.4°
        p_r = mav_pos - sphere_pos
        n_t_in_e_precise = -p_r / np.linalg.norm(p_r)
        z_1 = 1 - n_td_in_e.T.dot(n_t_in_e)
        if z_1 >= k_b:
            z_1 = k_b - 1e-3
        L_1 = 0.5 * np.log( k_b*k_b / (k_b*k_b - z_1*z_1) )
        K = z_1 / (k_b*k_b - z_1*z_1)
        # print("n_t_in_e_precise: {}, n_t_in_e: {}".format(n_t_in_e_precise, n_t_in_e))

        # L_2
        c_1 = 1.0
        z_2 = p_r
        v_r = mav_vel - sphere_vel
        # L_2 = 0.5 * z_2.T.dot(z_2)
        L_2 = L_1 + 0.5 * z_2.T.dot(z_2)
        # alpha_1 = -c_1 * p_r
        # alpha_1 = 10 * n_t_in_e_precise
        alpha_1 = 10. * n_t_in_e

        # L_3
        c_2 = 0.8
        z_3 = v_r - alpha_1
        f_drag = -C_d * mav_vel.T.dot(mav_vel)
        L_3 = L_2 + 0.5 * z_3.T.dot(z_3)
        # alpha_2 = -c_2*m*z_3 - m*g - c_1*m*v_r + m*n_t_in_e #- f_drag
        alpha_2 = -c_2*m*z_3 - m*g - c_1*m*v_r + m*n_t_in_e + K*m/np.linalg.norm(p_r)*(-np.identity(3)+n_t_in_e.dot(n_t_in_e.T)).dot(n_td_in_e)
        # print("eps:", m/np.linalg.norm(p_r)*(-np.identity(3)+n_t_in_e.dot(n_t_in_e.T)).dot(n_td_in_e))

        e_3 = np.array([[0], [0], [1]], dtype=np.float64)
        n_f = pos_info["mav_R"].dot(e_3)
        f_d = np.linalg.norm(alpha_2)
        thrust_d = f_d / m / np.linalg.norm(g) * thrust_hover

        # L_4
        c_3 = 0.3
        z_4 = 1. / m * (f_d * n_f - alpha_2)
        L_4 = L_3 + z_4.T.dot(z_4)

        a_r = mav_acc - sphere_acc
        z_2_dot = v_r
        z_3_dot = f_d/m*n_f + g + c_1*v_r #+ 1./m*f_drag
        alpha_2_dot = -c_2*m*z_3_dot - c_1*m*a_r - m*z_2_dot# + C_d*mav_vel.T.dot(mav_acc)
        n_f_x = self.skew(n_f)
        A = K*np.cross(n_td_in_e.T, n_t_in_e.T) + f_d / m * z_4.T.dot(n_f_x)
        B = z_4.T.dot(z_3 - 1./m*alpha_2_dot + c_3*z_4)

        # omega_in_e = m / f_d * np.linalg.pinv(n_f_x).dot(z_3 - 1./m*alpha_2_dot + c_3*z_4)
        omega_in_e = np.linalg.pinv(A).dot(B)
        omega_in_b = pos_info["mav_R"].T.dot(omega_in_e)

        # return control command
        return [omega_in_b[0], omega_in_b[1], omega_in_b[2], thrust_d]

    def WPController(self, pos_info, target_position_local):
        self.cnt_WP += 1

        direction = np.array([target_position_local[0] - pos_info["mav_pos"][0], target_position_local[1] - pos_info["mav_pos"][1]])
        direction /= np.linalg.norm(direction)
        v_horizontal = self.sat(self.cnt_WP * 0.05, self.v_norm_d) * direction
        yaw_d = math.atan2(direction[1], direction[0])
        cmd_yaw = self.yaw_control(yaw_d, pos_info["mav_yaw"], 0.2, 1.0)

        return [v_horizontal[0], v_horizontal[1], 0, cmd_yaw]

    def RotateHighspeedAttackController(self, pos_info, pos_i, image_center, controller_reset=False):
        if controller_reset: self.cnt = 0

        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc
        
        self.cnt += 1
        v_forward_base = pos_info["mav_R"].T.dot(pos_info["mav_vel"])[1]
        # v_m[1] = v_b[1] * 0.1/(1.01-cos_beta) + self.sat(self.cnt * 0.1,10)
        v_m = np.array([0., 0., 0.])
        # case1: (0.02, 3, 10)
        # case2: (0.05, 3, 12)
        v_m[1] = self.sat(self.cnt * 0.02, self.v_norm_d)
        # v_m[1] = self.v_norm_d
        v_m[0] = 15*v_b[0]
        v_m[2] = 12*v_b[2]
        # v_f = self.sat(self.cnt*0.02*np.array([0.,1.,0.]), 10)
        # v_m = (1-cos_beta)*v_b + (cos_beta)*v_f
        v = pos_info["mav_R"].dot(v_m)
        # v = self.sat(v, self.v_norm_d+5)
        yaw_rate = 0.002*(image_center[0] - pos_i[0])
        
        # print("v_b: {}\nv_m: {}\nv: {}".format(v_b, v_m, v))
        # print("yaw_rate: {}".format(yaw_rate))
        return [v[0], v[1], v[2], yaw_rate]

    #期望位置，反馈位置，位置比例系数，速读限幅
    def pos_control(self, target_pos, feb_pos, kp, sat_vel):
        err_pos = target_pos - feb_pos
        # cmd_pos_vel = self.sat(kp * err_pos, sat_vel)
        
        cmd_pos_vel = self.SatIntegral(kp * err_pos, sat_vel, -sat_vel)
        # cmd_pos_vel[2] = 
        return [cmd_pos_vel[0], cmd_pos_vel[1], cmd_pos_vel[2]]
        
    #期望位置，反馈位置，反馈角度，偏航角控制比例系数，角速度限幅
    def yaw_control(self, target_yaw, feb_yaw, kp_yaw, sat_yaw):
        #机头指向目标点的位置
        # desire_yaw = math.atan2(target_pos[1] - feb_pos[1], target_pos[0] - feb_pos[0])
        dlt_yaw = self.minAngleDiff(target_yaw, feb_yaw)
        cmd_yaw = self.Saturation(kp_yaw * dlt_yaw, sat_yaw, -sat_yaw)
        return cmd_yaw

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi

    # return a safty vz
    def SaftyZ(self, vz, satf):
        if vz > satf:
            return satf
        elif vz < -satf:
            return -satf
        else:
            return vz

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a

    def Saturation(self, a, up, down):
        if a > up:
            a = up
        elif a < down:
            a = down
        return a

    def PID_Control(self):
        p =1

    def Deadband(self, a, deadv):
        for i in range(len(a)):
            if abs(a[i]) < deadv:
                a[i] = 0
            elif a[i] > deadv or a[i] == deadv:
                a[i] = a[i] - deadv
            elif a[i] < - deadv or a[i] == - deadv:
                a[i] = a[i] + deadv
        return a
    
    def skew(self, n):
        return np.array([[0, -n[2], n[1]],\
                         [n[2], 0, -n[0]],\
                         [-n[1], n[0], 0]], dtype=np.float64)