# import required libraries
import time
import numpy as np
import cv2.cv2 as cv2
import sys

# import RflySim APIs
import PX4MavCtrlV4ROS as PX4MavCtrl
import VisionCaptureApi

vis = VisionCaptureApi.VisionCaptureApi()

# VisionCaptureApi 中的配置函数
vis.jsonLoad(1) # 加载Config.json中的传感器配置文件，强行设置为UDP取图

vis.startImgCap() # 开启取图

# Create MAVLink control API instance
mav = PX4MavCtrl.PX4MavCtrler(20100,'255.255.255.255')
# Init MAVLink data receiving loop
mav.InitMavLoop()

# create a ball, set its position and altitude, use the default red color
mav.sendUE4Pos(100,152,0,[3,0,-2],[0,0,0])
time.sleep(0.5)

# Function to calculate the location and radius of red ball
def calc_centroid(img):
    """Get the centroid and area of Red in the image"""
    low_range = np.array([0,0,80])
    high_range = np.array([100,100,255])
    th = cv2.inRange(img, low_range, high_range)
    dilated = cv2.dilate(th, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    cv2.imshow("dilated", dilated)
    cv2.waitKey(1)

    M = cv2.moments(dilated, binaryImage=True)
    if M["m00"] >= min_prop*width*height:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return [cx, cy, M["m00"]]
    else:
        return [-1, -1, -1]

# Function to obtain velocity commands for Pixhawk 
# according to the image processing results
def controller(p_i):
    # if the object is not in the image, search in clockwise
    if p_i[0] < 0 or p_i[1] < 0:
        return [0, 0, 0, 1]

    # found
    ex = p_i[0] - width / 2
    ey = p_i[1] - height / 2

    vx = 2 if p_i[2] < max_prop*width*height else 0
    vy = 0
    vz = K_z * ey
    yawrate = K_yawrate * ex
    
    # return forward, rightward, downward, and rightward-yaw 
    # velocity control sigals
    return [vx, vy, vz, yawrate]

ctrl_last=[0,0,0,0]
# Process image to obtain vehicle velocity control command
def procssImage():
    global ctrl_last
    if vis.hasData[0]:
        img_bgr=vis.Img[0]
        p_i = calc_centroid(img_bgr) 
        ctrl = controller(p_i) 
        ctrl_last=ctrl
    else:
        ctrl=ctrl_last
    return ctrl

# saturation function to limit the maximum velocity
def sat(inPwm,thres=1):
    outPwm= inPwm
    for i in range(len(inPwm)):
        if inPwm[i]>thres:
            outPwm[i] = thres
        elif inPwm[i]<-thres:
            outPwm[i] = -thres
    return outPwm


isEmptyData = False
lastTime = time.time()
startTime = time.time()
# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)
flag = 0

# parameters
width = 640
height = 480
channel = 4
min_prop = 0.000001
max_prop = 0.3
K_z = 0.003 * 640 / height
K_yawrate = 0.005 * 480 / width

num=0
lastClock=time.time()

# Start a endless loop with 30Hz, timeInterval=1/30.0
ctrlLast = [0,0,0,0]
while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime) # sleep until the desired clock
    else:
        lastTime = time.time()
    # The following code will be executed 30Hz (0.0333s)

    num=num+1
    if num%100==0:
        tiem=time.time()
        print('MainThreadFPS: '+str(100/(tiem-lastClock)))
        lastClock=tiem  

    if time.time() - startTime > 5 and flag == 0:
        # The following code will be executed at 5s
        print("5s, Arm the drone")
        mav.initOffboard()
        flag = 1
        mav.SendMavArm(True) # Arm the drone
        print("Arm the drone!, and fly to NED 0,0,-5")
        mav.SendPosNED(0, 0, -5, 0) # Fly to target position [0, 0, -5], i.e., take off to 5m

    if time.time() - startTime > 15 and flag == 1:
        flag = 2
         # The following code will be executed at 15s
        mav.SendPosNED(-30,-5, -5, 0)  # Fly to target position [-30,-5, -5]
        print("15s, fly to pos: -30,-5, -5")

    if time.time() - startTime > 25 and flag == 2:
        flag = 3
    # Show CV image and set the position
        if vis.hasData[0]:           
            img_bgr=vis.Img[0]
            cv2.imshow("dilated", img_bgr)
            cv2.waitKey(1)
            #time.sleep(0.5)
        
        print("25s, start to shoot the ball.")

    if time.time() - startTime > 25 and flag == 3:
        ctrlNow = procssImage()
        ctrl = sat(ctrlNow,5)
        # add a inertial component here to restrain the speed variation rate
        if ctrl[0]-ctrlLast[0] > 0.5:
            ctrl[0]=ctrlLast[0]+0.05
        elif ctrl[0]-ctrlLast[0] < -0.5:
            ctrl[0]=ctrlLast[0]-0.05
        if ctrl[1]-ctrlLast[1] > 0.5:
            ctrl[1]=ctrlLast[1]+0.05
        elif ctrl[1]-ctrlLast[1] < -0.5:
            ctrl[1]=ctrlLast[1]-0.05        
        ctrlLast = ctrl
        # if control signals is obtained, send to Pixhawk
        if not isEmptyData:
            mav.SendVelFRD(ctrl[0], ctrl[1], ctrl[2], ctrl[3])