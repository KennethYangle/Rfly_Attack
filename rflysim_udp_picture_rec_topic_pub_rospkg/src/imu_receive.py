#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket  #导入socket模块
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import rospy
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

      #server 接收端
      # 设置服务器默认端口号
#PORT = 31000
#PORT = 1000
      # 创建一个套接字socket对象，用于进行通讯
      # socket.AF_INET 指明使用INET地址集，进行网间通讯
      # socket.SOCK_DGRAM 指明使用数据协议，即使用传输层的udp协议


orientation_covariance_=[-1.0,0,0,0,0,0,0,0,0]
angular_velocity_covariance_=[0.01,0,0,0,0.01,0,0,0,0.01]
linear_acceleration_covariance_=[0.01,0,0,0,0.01,0,0,0,0.01]



class imuDataCopter:
    def __init__(self):
        self.checksum=1234567898
        self.seq=0
        self.timestmp=0
        self.acc=[0,0,0]
        self.rate=[0,0,0]


class SensorReqCopterSim:
    def __init__(self):
        self.checksum=12345
        self.sensorType=0
        self.updateFreq=100
        self.port=9996
        self.IP=[192,168,10,116]
        self.Params=[0,0,0,0,0,0]





if __name__ == '__main__':
	rospy.init_node('imu_recv')
        pub_topic_name = rospy.get_param('~pub_topic_name')
        IPpart1 = rospy.get_param('~IPpart1')
        IPpart2 = rospy.get_param('~IPpart2')
        IPpart3 = rospy.get_param('~IPpart3')
        IPpart4 = rospy.get_param('~IPpart4')
        port = rospy.get_param('~port')
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	#address = ("192.168.110.145", PORT) 
	#address = ("192.168.110.145", PORT) 
        IPadr=str(IPpart1)+"."+str(IPpart2)+"."+str(IPpart3)+"."+str(IPpart4)
        #address = (IPadr, PORT)
        address = (IPadr, port)
	server_socket.bind(address)  # 为服务器绑定一个固定的地址，ip和端口
	#server_socket.settimeout(10)  #设置一个时间提示，如果10秒钟没接到数据进行提示


	

	#rgb_pub=rospy.Publisher("/rgb_image_topic",Image, queue_size =3)
        #imu_pub=rospy.Publisher("/imu",Imu, queue_size =3)
        imu_pub=rospy.Publisher(pub_topic_name,Imu, queue_size =3)

	#rate = rospy.Rate(50)
        #sendImuReqCopterSim(copterID=1,freq=200,port=31000)
        copterID=1
        freq=200
        #port=31000
        srcs=SensorReqCopterSim()
        srcs.sensorType=0 #IMU传感器数据
        srcs.updateFreq=freq
        #if self.RemotSendIP!='':
            #cList = self.RemotSendIP.split('.')
            #if len(cList)==4:
                #srcs.IP[0]=int(cList[0])
                #srcs.IP[1]=int(cList[1])
                #srcs.IP[2]=int(cList[2])
                #srcs.IP[3]=int(cList[3])
        #srcs.IP=[192,168,10,116]
        srcs.IP[0]=int(IPpart1)
        srcs.IP[1]=int(IPpart2)
        srcs.IP[2]=int(IPpart3)
        srcs.IP[3]=int(IPpart4)
        srcs.port=port
        #sendReqToCopterSim(srcs,copterID) # 发送消息请求IMU数据
        u16Value=[srcs.checksum,srcs.sensorType,srcs.updateFreq,srcs.port]
	u8Value=srcs.IP
	fValue=srcs.Params
	#buf = struct.pack("4H4B6f",u16Value,u8Value,fValue)
        buf = struct.pack("4H4B6f",srcs.checksum,srcs.sensorType,srcs.updateFreq,srcs.port,srcs.IP[0],srcs.IP[1],srcs.IP[2],srcs.IP[3],srcs.Params[0],srcs.Params[1],srcs.Params[2],srcs.Params[3],srcs.Params[4],srcs.Params[5])
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	udp_socket.sendto(buf, ('255.255.255.255', 30100+(copterID-1)*2))





	    
	while not rospy.is_shutdown():
	    #buf,addr = server_socket.recvfrom(60000)  # 
            #try:
                buf,addr = server_socket.recvfrom(65500)
                if len(buf)==40:
                    print(len(buf[0:12]))
                    IMUData=struct.unpack('2i1d6f',buf)                   
                    if IMUData[0]==1234567898:
                        checksum=IMUData[0]
                        seq=IMUData[1]
                        timeStmp=IMUData[2]
                        acc0=IMUData[3]
                        acc1=IMUData[4]
                        acc2=IMUData[5]
                        rate0=IMUData[6]
                        rate1=IMUData[7]
                        rate2=IMUData[8] 

                        imgtimestampsec=math.floor(timeStmp)
                        imgtimestampnsec=(timeStmp-imgtimestampsec)*1000000000    
                        #if not self.hasIMUData:
                            #self.hasIMUData=True
                        print("Got CopterSim IMU Msg!")

                        imu_msg=Imu()
		        imu_msg.header.stamp.secs = imgtimestampsec
		        imu_msg.header.stamp.nsecs = imgtimestampnsec 
		        imu_msg.header.frame_id = "camera_imu_optical_frame"
		    
		        imu_msg.orientation.x = 0
		        imu_msg.orientation.y = 0
		        imu_msg.orientation.z = 0
		        imu_msg.orientation.w = 0
		   
		        imu_msg.angular_velocity.x = rate0
		        imu_msg.angular_velocity.y = rate1
		        imu_msg.angular_velocity.z = rate2
 
		        imu_msg.linear_acceleration.x = acc0
		        imu_msg.linear_acceleration.y = acc1
		        imu_msg.linear_acceleration.z = acc2

		        imu_msg.orientation_covariance[0]=orientation_covariance_[0]
		        imu_msg.orientation_covariance[1]=orientation_covariance_[1]
		        imu_msg.orientation_covariance[2]=orientation_covariance_[2]
		        imu_msg.orientation_covariance[3]=orientation_covariance_[3]
		        imu_msg.orientation_covariance[4]=orientation_covariance_[4]
		        imu_msg.orientation_covariance[5]=orientation_covariance_[5]
		        imu_msg.orientation_covariance[6]=orientation_covariance_[6]
		        imu_msg.orientation_covariance[7]=orientation_covariance_[7]
		        imu_msg.orientation_covariance[8]=orientation_covariance_[8]
		      
		      
		        imu_msg.angular_velocity_covariance[0]=angular_velocity_covariance_[0]
		        imu_msg.angular_velocity_covariance[1]=angular_velocity_covariance_[1]
		        imu_msg.angular_velocity_covariance[2]=angular_velocity_covariance_[2]
		        imu_msg.angular_velocity_covariance[3]=angular_velocity_covariance_[3]
		        imu_msg.angular_velocity_covariance[4]=angular_velocity_covariance_[4]
		        imu_msg.angular_velocity_covariance[5]=angular_velocity_covariance_[5]
		        imu_msg.angular_velocity_covariance[6]=angular_velocity_covariance_[6]
		        imu_msg.angular_velocity_covariance[7]=angular_velocity_covariance_[7]
		        imu_msg.angular_velocity_covariance[8]=angular_velocity_covariance_[8]
		      

		        imu_msg.linear_acceleration_covariance[0]=linear_acceleration_covariance_[0]
		        imu_msg.linear_acceleration_covariance[1]=linear_acceleration_covariance_[1]
		        imu_msg.linear_acceleration_covariance[2]=linear_acceleration_covariance_[2]
		        imu_msg.linear_acceleration_covariance[3]=linear_acceleration_covariance_[3]
		        imu_msg.linear_acceleration_covariance[4]=linear_acceleration_covariance_[4]
		        imu_msg.linear_acceleration_covariance[5]=linear_acceleration_covariance_[5]
		        imu_msg.linear_acceleration_covariance[6]=linear_acceleration_covariance_[6]
		        imu_msg.linear_acceleration_covariance[7]=linear_acceleration_covariance_[7]
		        imu_msg.linear_acceleration_covariance[8]=linear_acceleration_covariance_[8]
		      

		        imu_pub.publish(imu_msg)


                        #if isEnableRosTrans:
                           # pass
                            # 将IMU消息，推送到ROS消息中
                        
           # except:
                #print("Error to listen to IMU Msg!")
               # sys.exit(0)	    

	     
	    #rate.sleep()
     	#正常情况下接收数据并且显示，如果10秒钟没有接收数据进行提示（打印 "time out"）
	#当然可以不要这个提示，那样的话把"try:" 以及 "except"后的语句删掉就可以了


				      # 接收客户端传来的数据 recvfrom接收客户端的数据，默认是阻塞的，直到有客户端传来数据
				      # recvfrom 参数的意义，表示最大能接收多少数据，单位是字节
				      # recvfrom返回值说明
				      # receive_data表示接受到的传来的数据,是bytes类型
				      # client  表示传来数据的客户端的身份信息，客户端的ip和端口，元组
      #receive_data, client = server_socket.recvfrom(1024)
  

      #print("来自客户端%s,发送的%s\n" % (client, receive_data))  #打印接收的内容

