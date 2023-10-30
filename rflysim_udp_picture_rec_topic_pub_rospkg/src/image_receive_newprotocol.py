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

      # server 接收端
      # 设置服务器默认端口号
      # PORT = 9999
      # PORT = 1000
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


#	imgFlag = 1234567890
#	fhead_size = struct.calcsize('4i1d')
#	StartFlag = False
#	imgPackUnit = 60000
#	recvd_size = 0
#	data_total = b''


        CheckSum=1234567890
        fhead_size = struct.calcsize('4i1d')
        imgPackUnit = 60000
        
        seqList=[]
        dataList=[]
        timeList=[]
        recPackNum=0
        timeStmpStore=0


	ros_cv_bridge = CvBridge()

	#rgb_pub=rospy.Publisher("/rgb_image_topic",Image, queue_size =3)
        image_pub=rospy.Publisher(pub_topic_name,Image, queue_size =3)

	#rate = rospy.Rate(50)
	    
	while not rospy.is_shutdown():
            buf,addr = server_socket.recvfrom(imgPackUnit+2000) #加一些余量，确保包头数据考虑在内
            #print(len(buf))
            if len(buf)<fhead_size: #如果数据包还没包头长，数据错误
                continue
            dd = struct.unpack('4i1d',buf[0:fhead_size])#校验，包长度，包序号，总包数，时间戳
            #print(dd)
            if dd[0]!=CheckSum or dd[1]!=len(buf): #校验位不对或者长度不对
                print('Wrong Data!')
                continue
            packSeq=dd[2] #包序号
            if packSeq==0:#如果是第一个包
                seqList=[] # 清空数据序号列表
                dataList=[] #清空数据缓存列表
                seqList=seqList+ [packSeq]#提取序号
                dataList=dataList+[buf[fhead_size:]] #提取包头剩余数据
                timeStmpStore = dd[4]#提取时间戳
                recPackNum=dd[3] #以包头定义的总包作为接收结束标志
            else: #如果不是包头，直接将其存入列表
                if recPackNum==0:
                    continue     
                #因为这两句运行时会报错所以注释掉
                #if not math.isclose(timeStmpStore,dd[4],rel_tol=0.00001): #如果时间戳不一致
                #    continue #跳过这个包
                #下面是换一种写法实现上面两句代码相同的逻辑，进行时间戳的比对判断。
                if abs(dd[4]-timeStmpStore)>0.00001:
                     continue
                seqList=seqList+ [packSeq]#提取序号
                dataList=dataList+[buf[fhead_size:]] #提取包头剩余数据
            #if typeID==2:
                #print(seqList,recPackNum,len(dataList))
            if len(seqList)==recPackNum: #如果收到的包达到总数了，开始处理图像
                recPackNum=0
                #print('Start Img Cap')
                data_total=b''
                dataOk=True
                for i in range(len(seqList)):
                    if seqList.count(i)<1:
                        dataOk=False # 如果某序号不在包中，报错
                        print('Failed to process img pack')
                        break                        
                    idx0 = seqList.index(i) #按次序搜索包序号
                    data_total=data_total+dataList[idx0]
                #if typeID==2:
                #    print(len(data_total))
                if dataOk: #如果数据都没问题，开始处理图像
                    nparr = np.frombuffer(data_total, np.uint8)
                    if TYPE == 'rgb':
                        colorType=cv2.IMREAD_COLOR
                    if TYPE == 'depth':
                        colorType=cv2.IMREAD_ANYDEPTH
                    if TYPE == 'gray':
                        colorType=cv2.IMREAD_GRAYSCALE
                    Img = cv2.imdecode(nparr, colorType)
                    #print(len(Img))
                    print('True Data.\n')
                    #image_msg = ros_cv_bridge.cv2_to_imgmsg(Img, encoding="bgr8")
                    image_msg = ros_cv_bridge.cv2_to_imgmsg(Img, encoding=ENCODING)
                    #image_msg.header.frame_id = "camera_color_optical_frame"
                    image_msg.header.frame_id = FRAME_ID
                    imgtimestampsec=math.floor(timeStmpStore)
                    imgtimestampnsec=(timeStmpStore-imgtimestampsec)*1000000000
                    image_msg.header.stamp.secs = imgtimestampsec
                    image_msg.header.stamp.nsecs = imgtimestampnsec
                    image_pub.publish(image_msg)
                    #cv2.imshow("caisetu",Img)
		    #cv2.waitKey(1) #1就能连续流畅弹出图片，0的话弹出一个得手动关掉程序才继续运行再弹出下一个。

	    #rate.sleep()



 
	  # 接收客户端传来的数据 recvfrom接收客户端的数据，默认是阻塞的，直到有客户端传来数据
	  # recvfrom 参数的意义，表示最大能接收多少数据，单位是字节
	  # recvfrom返回值说明
	  # receive_data表示接受到的传来的数据,是bytes类型
	  # client  表示传来数据的客户端的身份信息，客户端的ip和端口，元组
          # receive_data, client = server_socket.recvfrom(1024)
          # print("来自客户端%s,发送的%s\n" % (client, receive_data))  #打印接收的内容

