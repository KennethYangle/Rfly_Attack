# -*- coding: utf-8 -*-
import cv2
import sys
import time
import VisionCaptureApi
import os
# The IP should be specified by the other computer
vis = VisionCaptureApi.VisionCaptureApi()

# Send command to UE4 Window 1 to change resolution 
# vis.sendUE4Cmd(b'r.setres 720x405w',0) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
vis.sendUE4Cmd(b't.MaxFPS 30',0) # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(0.5)    
vis.sendUE4Cmd(b'r.setres 720x405w',1) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
vis.sendUE4Cmd(b't.MaxFPS 30',1) # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(0.5)    
vis.sendUE4Cmd(b'r.setres 720x405w',2) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
vis.sendUE4Cmd(b't.MaxFPS 30',2) # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(0.5)    
vis.sendUE4Cmd(b'r.setres 720x405w',3) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
vis.sendUE4Cmd(b't.MaxFPS 30',3) # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(0.5)    

# VisionCaptureApi 中的配置函数
vis.jsonLoad() # 加载Config.json中的传感器配置文件

# vis.RemotSendIP = '192.168.110.145'
# 注意，手动修改RemotSendIP的值，可以将图片发送到远端Linux电脑的IP地址
# 如果不修改这个值，那么发送的IP地址为json文件中SendProtocol[1:4]定义的IP
# 图片的发送端口，为json中SendProtocol[5]定义好的。

vis.isUE4DirectUDP=True
# 注意，手动修改本命令能强制将图片发送机制为UE4直接发出UDP图片到指定IP地址
# 如果不修改这个值，那么发送机制由json文件中SendProtocol[0]中定义

isSuss = vis.sendReqToUE4() # 向RflySim3D发送取图请求，并验证
if not isSuss: # 如果请求取图失败，则退出
    sys.exit(0)


#注意：这里不需要调用startImgCap()来去图，本程序只是发送数据请求
print('Image UDP direct command has sent to RflySim3D')

# create a ball, set its position and altitude, use the default red color
# vis.sendUE4Pos(100,152,0,[20,10,-1.5],[0,0,0])
# vis.sendUE4Pos(101,152,0,[23,-15,-2],[0,0,0])
# time.sleep(0.5)
