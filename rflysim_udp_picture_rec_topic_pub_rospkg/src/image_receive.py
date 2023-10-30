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

      #server 接收端
      # 设置服务器默认端口号
#PORT = 9999
#PORT = 1000
      # 创建一个套接字socket对象，用于进行通讯
      # socket.AF_INET 指明使用INET地址集，进行网间通讯
      # socket.SOCK_DGRAM 指明使用数据协议，即使用传输层的udp协议



if __name__ == '__main__':
	rospy.init_node('image_recv')
        port = rospy.get_param('~port')
        IP = rospy.get_param('~IP')
        pub_topic_name = rospy.get_param('~pub_topic_name')
        TYPE = rospy.get_param('~type')
        FRAME_ID = rospy.get_param('~frame_id')
        ENCODING = rospy.get_param('~encoding')
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	#address = ("192.168.110.145", PORT) 
	#address = ("192.168.110.145", PORT) 
        #address = ("192.168.110.145", port)
        address = (IP, port)
	server_socket.bind(address)  # 为服务器绑定一个固定的地址，ip和端口
	#server_socket.settimeout(10)  #设置一个时间提示，如果10秒钟没接到数据进行提示


	imgFlag = 1234567890
	fhead_size = struct.calcsize('4i1d')
	StartFlag = False
	imgPackUnit = 60000
	recvd_size = 0
	data_total = b''

	ros_cv_bridge = CvBridge()

	#rgb_pub=rospy.Publisher("/rgb_image_topic",Image, queue_size =3)
        rgb_pub=rospy.Publisher(pub_topic_name,Image, queue_size =3)

	#rate = rospy.Rate(50)
	    
	while not rospy.is_shutdown():
	    buf,addr = server_socket.recvfrom(60000)  # 
	    #print('get message.\n')
	    #if len(buf)==fhead_size:
	    if len(buf)==24:
		#print('get head.\n')
		dd = struct.unpack('4i1d',buf)
		chksm = dd[0]
		if chksm==imgFlag:
		    #print('get right head.\n')
		    StartFlag=True
		    data_size = dd[1]
		    imgPackUnit= dd[2]
		    recvd_size = 0
		    data_total = b''
		    timeStmp = dd[4]
                    imgtimestampsec=math.floor(timeStmp)
                    imgtimestampnsec=(timeStmp-imgtimestampsec)*1000000000
		    continue

	    #后面相当于循环接收图片码流
	    if StartFlag:
		if data_size -recvd_size >imgPackUnit:
		    recvd_size += len(buf)
		else:
		    recvd_size = data_size  
		data_total += buf
		            
		if recvd_size == data_size:
		    nparr = np.frombuffer(data_total, np.uint8)
		    #colorType=cv2.IMREAD_COLOR
                    if TYPE == 'rgb':
                        colorType=cv2.IMREAD_COLOR
                    if TYPE == 'depth':
                        colorType=cv2.IMREAD_ANYDEPTH
                    if TYPE == 'gray':
                        colorType=cv2.IMREAD_GRAYSCALE
		    Img = cv2.imdecode(nparr, colorType)
		    print('True Data.\n')
                    #image_msg = ros_cv_bridge.cv2_to_imgmsg(Img, encoding="bgr8")
                    image_msg = ros_cv_bridge.cv2_to_imgmsg(Img, encoding=ENCODING)
                    #image_msg.header.frame_id = "camera_color_optical_frame"
                    image_msg.header.frame_id = FRAME_ID
                    image_msg.header.stamp.secs = imgtimestampsec
                    image_msg.header.stamp.nsecs = imgtimestampnsec
                    rgb_pub.publish(image_msg)
		    hasData = True
		    StartFlag=False
		    #cv2.imshow("caisetu",Img)
		    #cv2.waitKey(1) #1就能连续流畅弹出图片，0的话弹出一个得手动关掉程序才继续运行再弹出下一个。
		       
		    if recvd_size>data_size:
		        print('Wrong Data.')
		        StartFlag=False

	     
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

