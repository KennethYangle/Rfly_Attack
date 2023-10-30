#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State,PositionTarget,ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, CommandLong
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
import math
import os
import socket
import threading
import struct
import sys

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

import subprocess
#from pymavlink.dialects.v20 import common as mavlink2


class PX4MavCtrler:
    #Create a new MAVLink communication instance, 
    # For UDP connect
    # use format PX4MavCtrler(port,IP), e.g., PX4MavCtrler(20100,'127.0.0.1')
    # For hardware connection
    #Windows use format PX4MavCtrler('COM3') or PX4MavCtrler('COM3:115200') for Pixhawk USB port connection
    #Windows use format 'COM4:57600' for Pixhawk serial port connection
    #Linux use format '/dev/ttyUSB0' or '/dev/ttyUSB0:115200' for USB, or '/dev/ttyAMA0:57600' for Serial port (RaspberryPi example)
    # constructor function
    def __init__(self, port=20100,ip='127.0.0.1'):
        
        self.isCom=False
        self.baud=115200
        self.tgtSys=1
        if(isinstance(port, str)):
            self.isCom=True
            strlist = port.split(':')
            
            if(len(strlist)>=2):
                if strlist[1].isdigit():
                    self.baud=int(strlist[1])
                port=strlist[0]
               
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        if ip == '255.255.255.255':
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.ip = ip  # 服务器ip和端口
        self.port = port
        self.child = None        
        
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_vel = None
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None
        self.command = TwistStamped()
        self.offCmd = PositionTarget()
        self.offCmd.header.frame_id = "world"
        
        self.isInOffboard = False
        
        self.uavAngEular = [0, 0, 0]
        self.uavAngRate = [0, 0, 0]
        self.uavPosNED = [0, 0, 0]
        self.uavVelNED = [0, 0, 0]
        
        
        self.count = 0
        
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.local_vel_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.vel_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.setparamService = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.sendCmdLongService = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        print("Px4 Controller Initialized!")


    # def start(self):
    #     rospy.init_node("offboard_node")
    #     rate = rospy.Rate(20)

    #     for i in range(10):
    #         self.vel_pub.publish(self.command)
    #         self.arm_state = self.arm()
    #         self.offboard_state = self.offboard()
    #         rate.sleep()

    #     start_time = rospy.Time.now()
    #     '''
    #     main ROS thread
    #     '''
    #     while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
    #         if rospy.Time.now() - start_time < rospy.Duration(5):
    #             self.command.twist.linear.x = 0
    #             self.command.twist.linear.z = 2
    #             self.command.twist.angular.z = 0
    #         elif rospy.Time.now() - start_time < rospy.Duration(20):
    #             self.command.twist.linear.x = 2
    #             self.command.twist.linear.z = 0
    #             self.command.twist.angular.z = 0.1
    #         else:
    #             self.command.twist.linear.x = 0
    #             self.command.twist.linear.z = -1
    #             self.command.twist.angular.z = 0

    #         self.vel_pub.publish(self.command)
    #         rate.sleep()

    def InitMavLoop(self):
        
        if self.isCom:
            the_connection = mavutil.mavlink_connection(self.port,self.baud)
            the_connection.recv_match(
                    type=['HEARTBEAT'],
                    blocking=True)
            self.tgtSys=the_connection.target_system
            the_connection.close()
            print(self.tgtSys)
            
            cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="serial://'+self.port+':'+str(self.baud)+'"'
        else:
            
            the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))
            
            the_connection.recv_match(
                    type=['HEARTBEAT'],
                    blocking=True)
            self.tgtSys=the_connection.target_system
            the_connection.close()
            print(self.tgtSys)
            
            cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+str(self.port)+'"'
        
        print(cmdStr)
        self.child = subprocess.Popen(cmdStr,
                         shell=True,
                         stdout=subprocess.PIPE)

        time.sleep(10)
        print(self.child.poll())


    def sendStartMsg(self,copterID=-1):
        buf = struct.pack("3i",1234567890,1,copterID)
        self.udp_socket.sendto(buf, ('224.0.0.10', 20007)) 
        print("Send start Msg")
        time.sleep(0.03)

    def waitForStartMsg(self):
        MYPORT = 20007
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind((ANY,MYPORT))
        status = sock.setsockopt(socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
    
        sock.setblocking(1)
        #ts = time.time()
        
        print("Waiting for start Msg")
        while True:
            try:
                buf,addr = sock.recvfrom(65500)
                if len(buf)==12:
                    #print(len(buf[0:12]))
                    checksum,isStart,ID=struct.unpack('3i',buf)
                    if checksum==1234567890 and isStart:
                        if ID<0 or ID==self.CopterID:
                            print('Got start Msg, continue to run.')
                            break
            except:
                print("Error to listen to Start Msg!")
                sys.exit(0)


    def SendMavArm(self, isArm):
        if self.armService(isArm):
            return True
        else:
            if isArm:
                print("Vehicle disarming failed!")
            else:
                print("Vehicle arming failed!")
            return False        

    def initOffboard(self):
        self.t2 = threading.Thread(target=self.OffboardLoop, args=())
        self.isInOffboard = True        
        print("Offboard Started.")
        rospy.init_node("offboard_node")
        rate = rospy.Rate(20)
        self.SendVelNED(0, 0, 0, 0) 
        
        # if not self.hasSendDisableRTLRC:
        #     #self.sendMavSetParam('NAV_RCL_ACT',0,'INT')
        #     #self.sendMavSetParam('NAV_DLL_ACT', 0, 'INT')
        #     self.hasSendDisableRTLRC = True
        
        for i in range(10):
            self.offCmd.header.stamp = rospy.Time.now()
            self.offCmd.header.seq = self.count
            self.count = self.count+1            
            self.vel_raw_pub.publish(self.offCmd)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rate.sleep()
        self.t2.start()

    def OffboardLoop(self):
        rate = rospy.Rate(30)
        while True:
            if not self.isInOffboard:
                break
            if self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
                self.offCmd.header.stamp = rospy.Time.now()
                self.offCmd.header.seq = self.count
                self.count = self.count+1          
                self.vel_raw_pub.publish(self.offCmd)
                #print('Send offboard msg.')
                #print(self.offCmd)
                #print(self.uavPosNED)
                rate.sleep()
            else:
                break
        print("Offboard Stoped.")

    def endOffboard(self):
        self.isInOffboard = False
        self.t2.join()

    def stopRun(self):
        self.child.kill()
        self.child.terminate()
        print('Please close all Terminal windows to close')
        
    def calcTypeMask(self,EnList):
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        return y

    def SendVelNED(self,vx,vy,vz,yawrate):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = vx
        self.offCmd.velocity.y = -vy
        self.offCmd.velocity.z = -vz
        self.offCmd.yaw_rate = -yawrate


    def SendVelFRD(self,vx,vy,vz,yawrate):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = vx
        self.offCmd.velocity.y = -vy
        self.offCmd.velocity.z = -vz
        self.offCmd.yaw_rate = -yawrate


    def SendPosNED(self,x,y,z,yaw):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([1,0,0,0,1,0])
        self.offCmd.position.x = x
        self.offCmd.position.y = -y
        self.offCmd.position.z = -z
        self.offCmd.yaw = -yaw + math.pi/2

    def SendPosFRD(self,x,y,z,yaw):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([1,0,0,0,1,0])
        self.offCmd.position.x = x
        self.offCmd.position.y = -y
        self.offCmd.position.z = -z
        self.offCmd.yaw = -yaw + math.pi/2
            

    def local_pose_callback(self, msg):
        self.local_pose = msg
        ang = self.q2Euler(self.local_pose.pose.orientation)

        self.uavAngEular[0] = ang[1]
        self.uavAngEular[1] = ang[0]
        self.uavAngEular[2] = self.yawSat(-ang[2]+math.pi/2)
         
        self.uavPosNED[0]=self.local_pose.pose.position.y
        self.uavPosNED[1]=self.local_pose.pose.position.x
        self.uavPosNED[2]=-self.local_pose.pose.position.z
        
    def local_vel_callback(self, msg):    
        self.local_vel = msg
        
        self.uavVelNED[0] = self.local_vel.twist.linear.y
        self.uavVelNED[1] = self.local_vel.twist.linear.x
        self.uavVelNED[2] = -self.local_vel.twist.linear.z
        
        self.uavAngRate[0] = self.local_vel.twist.angular.y
        self.uavAngRate[1] = self.local_vel.twist.angular.x
        self.uavAngRate[2] = -self.local_vel.twist.angular.z        
        
        

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    def q2Euler(self,q):
        w,x,y,z= q.w, q.x, q.y, q.z
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(2*(w*y-z*x))
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        return [roll,pitch,yaw]

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False
        
    def yawSat(self,yaw):
        yawOut=yaw
        if yaw>math.pi/2:
            yawOut=yaw-math.pi/2
        if yaw<-math.pi/2:
            yawOut=yaw+math.pi/2
        return yawOut
        



    # struct Ue4CMD{
    #     int checksum;
    #     char data[52];
    # }
    # send command to control the display style of RflySim3D
    def sendUE4Cmd(self,cmd,windowID=-1):
        buf = struct.pack("i52s",1234567890,cmd)
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send



    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos2Ground(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        buf = struct.pack("3i7f",1234567891,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send


    def sendUE4PosScale(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale=[1,1,1],windowID=-1):
        buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send

    def sendUE4PosScale2Ground(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale=[1,1,1],windowID=-1):
        buf = struct.pack("3i10f",1234567891,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send



    #struct Multi3DData100New {
    #    int checksum;
    #    uint16 copterID[100];
    #    uint16 vehicleType[100];
    #    float PosE[300];
    #    float AngEuler[300];
    #    uint16 Scale[300];
    #    float MotorRPMSMean[100];
    #}
    #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
    #copterID是100维整型数组，飞机ID，这个从1给到100即可,如果ID给0则不显示次飞机
    #vehicleType是100维整型数组，飞机类型，四旋翼给3即可
    #MotorRPMSMean是100维数组，飞机螺旋桨转速，单位RPM，四旋翼默认给1000即可
    #PosE是300维数组，飞机位置，可以随机生成，单位是m
    #AngEuler是300维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
    #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
    def sendUE4PosScale100(self,copterID,vehicleType,PosE,AngEuler,MotorRPMSMean,Scale,isFitGround=False,windowID=-1):
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i200H600f300H100f",checksum,*copterID,*vehicleType,*PosE,*AngEuler,*Scale,*MotorRPMSMean)
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send


    # struct Multi3DData2New {
    #   int checksum;
    # 	uint16 copterID[20];
    # 	uint16 vehicleType[20];
    # 	float PosE[60];
    # 	float AngEuler[60];
    # 	uint16 Scale[60];
    # 	float PWMs[160];
    # }
    #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
    #copterID是20维整型数组，飞机ID，这个从1给到20即可,如果ID给0则不显示次飞机
    #vehicleType是20维整型数组，飞机类型，四旋翼给3即可
    #PosE是60维数组，飞机位置，可以随机生成，单位是m
    #AngEuler是60维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
    #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
    #PWMs是160维数组，对应20个飞机各8个旋翼转速，单位RPM，四旋翼默认给1000即可

    def sendUE4PosScalePwm20(self,copterID,vehicleType,PosE,AngEuler,Scale,PWMs,isFitGround=False,windowID=-1):
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i40H120f60H160f",checksum,*copterID,*vehicleType,*PosE,*AngEuler,*Scale,*PWMs)
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
            
            
    def sendRebootPix(self,copterID,delay=-1):
        checksum = 987654321 #checksum for reboot message
        buf = struct.pack("2i",checksum,delay)
        self.udp_socket.sendto(buf, ('255.255.255.255', 20100+copterID*2-2))


    # struct SOut2Simulator {
    #     int copterID;  //Vehicle ID
    #     int vehicleType;  //Vehicle type
    #     double runnedTime; //Current Time stamp (s)
    #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
    #     float PosE[3];   //NED vehicle position in earth frame (m)
    #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
    #     float AngQuatern[4]; //Vehicle attitude in Quaternion
    #     float MotorRPMS[8];  //Motor rotation speed (RPM)
    #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
    #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
    #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)
        
    # }
    # typedef struct _netDataShort {
    #     int tg;
    #     int        len;
    #     char       payload[192];
    # }netDataShort;
    
    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(self,copterID,vehicleType,MotorRPMS,VelE,PosE,RateB,AngEuler,windowID=-1):
        runnedTime = time.time()-self.startTime
        #VelE=[0,0,0]
        AngQuatern=[0,0,0,0]
        AccB=[0,0,0]
        #RateB=[0,0,0]
        PosGPS=[0,0,0]
        # buf for SOut2Simulator, len=152
        buf0 = struct.pack("2i1d27f3d",copterID,vehicleType,runnedTime,*VelE,*PosE,*AngEuler,*AngQuatern,*MotorRPMS,*AccB,*RateB,*PosGPS)
        # buf for remaining 192-152=40bytes of payload[192] of netDataShort
        buf1 = bytes([0]*(192-len(buf0)))
        # buf for tg and len in netDataShort
        buf2 = struct.pack("2i",2,len(buf0))
        # buf for netDataShort
        buf=buf2+buf0+buf1
        #print(len(buf))
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')
        
        


    # //输出到模拟器的数据
    # struct SOut2SimulatorSimpleTime {
    #     int checkSum;
    #     int copterID;  //Vehicle ID
    #     int vehicleType;  //Vehicle type
    #     float PWMs[8];
    #     float PosE[3];   //NED vehicle position in earth frame (m)
    #     float VelE[3];
    #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
    #     double runnedTime; //Current Time stamp (s)
    # };
    #struct.pack 3i1d17f
    
    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosSimple(self,copterID,vehicleType,PWMs,VelE,PosE,AngEuler,windowID=-1):
        runnedTime = time.time()-self.startTime
        checkSum=1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i17f1d",checkSum,copterID,vehicleType,*PWMs,*PosE,*VelE,*AngEuler,runnedTime)
        #print(len(buf))
        if windowID<0:
            if self.ip=='127.0.0.1':
                self.udp_socket.sendto(buf, ('224.0.0.11', 20008)) #multicast address, send to all RflySim3Ds on this PC
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            if self.ip!='127.0.0.1' and self.ip!='255.255.255.255':
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) #specify PC's IP to send
        #print('Message Send')
            
            
    def sendMavSetParam(self,param_id_str, param_value, param_type):
        # param=ParamSet
        # print(param)
        # param.param_id=param_id_str
        # if param_type=='INT':
        #     param.value.integer=param_value
        # else:
        #     param.value.real=param_value
        # if param_type=='INT':
        #     #self.setparamService({"param_id":param_id_str,"value":{"integer":param_value}})
        #     self.setparamService({param_id=param_id_str,value.integer=param_value})
        # else:
        #     #self.setparamService({"param_id":param_id_str,"value":{"real":param_value}})
        #     self.setparamService({param_id=param_id_str,value.real=param_value})
        
        if param_type=='INT':
            val = ParamValue(integer=param_value, real=0)
        else:
            val = ParamValue(integer=0, real=param_value)
        
        self.setparamService(param_id=param_id_str, value=val)
        
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        self.sendCmdLongService(broadcast=False,confirmation=0,command=command,param1=param1,param2=param2,param3=param3,param4=param4,param5=param5,param6=param6,param7=param7)
