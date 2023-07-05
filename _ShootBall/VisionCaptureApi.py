import socket
import threading
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import math
import copy

#是否启用ROS图像转发功能
#IsEnable ROS image topic forwarding
isEnableRosTrans=False

if isEnableRosTrans:
    import rospy
    # 加入其他的ROS库


class VisionSensorReq:
    """ This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
        # struct VisionSensorReq {
        # 	uint16 checksum; //数据校验位，12345
        # 	uint16 SeqID; //内存序号ID
        # 	uint16 TypeID; //传感器类型ID
        # 	uint16 TargetCopter; //绑定的目标飞机     //可改变
        # 	uint16 TargetMountType; //绑定的类型    //可改变
        # 	uint16 DataWidth;   //数据或图像宽度
        # 	uint16 DataHeight; //数据或图像高度
        # 	uint16 DataCheckFreq; //检查数据更新频率
        # 	uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
        # 	float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
        # 	float SensorPosXYZ[3]; // 传感器安装位置    //可改变
        # 	float SensorAngEular[3]; //传感器安装角度   //可改变
        # 	float otherParams[8]; //预留的八位数据位
        # }16H15f
    """
    def __init__(self):
        self.checksum=12345
        self.SeqID=0
        self.TypeID=1
        self.TargetCopter=1
        self.TargetMountType=0
        self.DataWidth=0
        self.DataHeight=0
        self.DataCheckFreq=0
        self.SendProtocol=[0,0,0,0,0,0,0,0]
        self.CameraFOV=90
        self.SensorPosXYZ=[0,0,0]
        self.SensorAngEular=[0,0,0]
        self.otherParams=[0,0,0,0,0,0,0,0]


class imuDataCopter:
    """ This is a class (C++ struct) for IMU data receive from CopterSim
        # struct imuDataCopter{
        #     int checksum; //数据校验位1234567898
        #     int seq; //消息序号
        #     double timestmp;//时间戳
        #     float acc[3];
        #     float rate[3];
        # }   //2i1d6f    
    """
    def __init__(self):
        self.checksum=1234567898
        self.seq=0
        self.timestmp=0
        self.acc=[0,0,0]
        self.rate=[0,0,0]


class SensorReqCopterSim:
    """This is a class (C++ struct) that sent to UE4 to request sensor data.
        # struct SensorReqCopterSim{
        #     uint16_t checksum;
        #     uint16_t sensorType;
        #     uint16_t updateFreq;
        #     uint16_t port;
        #     uint8_t IP[4];
        #     float Params[6];
        # } //4H4B6f    
    """
    def __init__(self):
        self.checksum=12345
        self.sensorType=0
        self.updateFreq=100
        self.port=9998
        self.IP=[127,0,0,1]
        self.Params=[0,0,0,0,0,0]


class VisionCaptureApi:
    """ This is the API class for python to request image from UE4
    """
    def __init__(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.VisSensor = []
        self.Img=[]
        self.hasData=[]
        self.timeStmp=[]
        self.IpList=[]
        self.portList=[]
        self.hasReqUE4=False
        self.sleepCheck=0.005
        self.ip='127.0.0.1'
        self.isRemoteSend=False
        self.RemotSendIP=''
        self.isUE4DirectUDP=False
        self.imu=imuDataCopter()
        self.hasIMUData=False
        if isEnableRosTrans:
            pass
            # 加入ROS节点的初始化工作
        

    def addVisSensor(self,vsr=VisionSensorReq()):
        """ Add a new VisionSensorReq struct to the list
        """
        if isinstance(vsr,VisionSensorReq):
            self.VisSensor = self.VisSensor + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")
        
    def sendUE4Cmd(self,cmd,windowID=-1):
        """send command to control the display style of RflySim3D
            The available command are list as follows, the command string is as b'RflyShowTextTime txt time'
            RflyShowTextTime(String txt, float time)\\ let UE4 show txt with time second
            RflyShowText(String txt)\\  let UE4 show txt 5 second
            RflyChangeMapbyID(int id)\\ Change the map to ID (int number)
            RflyChangeMapbyName(String txt)\\ Change to map with name txt
            RflyChangeViewKeyCmd(String key, int num) \\ the same as press key + num on UE4
            RflyCameraPosAngAdd(float x, float y, float z,float roll,float pitch,float yaw) \\ move the camera with x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflyCameraPosAng(float x, float y, float z, float roll, float pitch, float yaw) \\ set the camera with x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyCameraFovDegrees(float degrees) \\ change the cameras fov (degree)
            RflyChange3DModel(int CopterID, int veTypes=0) \\ change the vehicle 3D model to ID
            RflyChangeVehicleSize(int CopterID, float size=0) \\change vhielce's size
            RflyMoveVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ move the vehicle's  x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflySetVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ set the vehilce's x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyScanTerrainH(float xLeftBottom(m), float yLeftBottom(m), float xRightTop(m), float yRightTop(m), float scanHeight(m), float scanInterval(m)) \\ send command to let UE4 scan the map to generate png and txt files
            RflyCesiumOriPos(double lat, double lon, double Alt) \\ change the lat, lon, Alt (degrees) of the Cesium map origin
            RflyClearCapture \\ clear the image capture unit
            struct Ue4CMD{
                int checksum;
                char data[52];
            }
        """
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

    def sendUE4Pos(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            struct SOut2SimulatorSimple {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            }  3i7f
        """
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
    def sendUE4Pos2Ground(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states on the ground
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            }  3i7f
        """
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


    def sendUE4PosScale(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],Scale=[1,1,1],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
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

    def sendUE4PosScale2Ground(self,copterID=1,vehicleType=3,MotorRPMSMean=0,PosE=[0,0,0],AngEuler=[0,0,0],Scale=[1,1,1],windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
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


    def sendUE4PosScale100(self,copterID,vehicleType,PosE,AngEuler,MotorRPMSMean,Scale,isFitGround=False,windowID=-1):
        """send the position & angle information to RflySim3D to create 100 vehicles once
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
        """
        
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


    def sendUE4PosScalePwm20(self,copterID,vehicleType,PosE,AngEuler,Scale,PWMs,isFitGround=False,windowID=-1):
        """send the position & angle information to RflySim3D to create 20 vehicles once
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
        """
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
        """ Send message to restart simulation to CopterID
        """
        checksum = 987654321 #checksum for reboot message
        buf = struct.pack("2i",checksum,delay)
        self.udp_socket.sendto(buf, ('255.255.255.255', 20100+copterID*2-2))

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(self,copterID,vehicleType,MotorRPMS,VelE,PosE,RateB,AngEuler,windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
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
        
        """
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
        
        
    def sendUE4PosSimple(self,copterID,vehicleType,PWMs,VelE,PosE,AngEuler,windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
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
        """
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


    def sendReqToCopterSim(self,srcs=SensorReqCopterSim(),copterID=1):
        """ send UDP message SensorReqCopterSim to CopterSim to request a sensor data
        the copterID specify the index of CopterSim to request
        """
        if type(srcs).__name__ != 'SensorReqCopterSim':
            print('Error: input is not SensorReqCopterSim class')
            return
        u16Value=[srcs.checksum,srcs.sensorType,srcs.updateFreq,srcs.port]
        u8Value=srcs.IP
        fValue=srcs.Params
        buf = struct.pack("4H4B6f",*u16Value,*u8Value,*fValue)
        self.udp_socket.sendto(buf, ('255.255.255.255', 30100+(copterID-1)*2))


    def sendImuReqCopterSim(self,copterID=1,IP='127.0.0.1',port=31000,freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP that copterSim send back to
        port is the port that CopterSim send to 
        freq is the frequency of the send data
        """
        srcs=SensorReqCopterSim()
        srcs.sensorType=0 #IMU传感器数据
        srcs.updateFreq=freq
        if IP!='':
            cList = IP.split('.')
            if len(cList)==4:
                srcs.IP[0]=int(cList[0])
                srcs.IP[1]=int(cList[1])
                srcs.IP[2]=int(cList[2])
                srcs.IP[3]=int(cList[3])
        srcs.port=port+copterID-1
        self.sendReqToCopterSim(srcs,copterID) # 发送消息请求IMU数据
        self.udp_imu.bind(('0.0.0.0',srcs.port))
        self.tIMU = threading.Thread(target=self.getIMUDataLoop, args=())
        self.tIMU.start()


    def getIMUDataLoop(self):
        print("Start lisening to IMU Msg")
        while True:
            try:
                buf,addr = self.udp_imu.recvfrom(65500)
                if len(buf)==40:
                    #print(len(buf[0:12]))
                    IMUData=struct.unpack('2i1d6f',buf)                   
                    if IMUData[0]==1234567898:
                        self.imu.checksum=IMUData[0]
                        self.imu.seq=IMUData[1]
                        self.imu.timestmp=IMUData[2]
                        self.imu.acc[0]=IMUData[3]
                        self.imu.acc[1]=IMUData[4]
                        self.imu.acc[2]=IMUData[5]
                        self.imu.rate[0]=IMUData[6]
                        self.imu.rate[1]=IMUData[7]
                        self.imu.rate[2]=IMUData[8]     
                        if not self.hasIMUData:
                            self.hasIMUData=True
                            print("Got CopterSim IMU Msg!")
                        if isEnableRosTrans:
                            pass
                            # 将IMU消息，推送到ROS消息中
                        
            except:
                print("Error to listen to IMU Msg!")
                sys.exit(0)


    def sendUpdateUEImage(self,vs=VisionSensorReq(),windID=0):
        if not isinstance(vs,VisionSensorReq):
            raise Exception("Wrong data input to addVisSensor()")
        intValue=[vs.checksum,vs.SeqID,vs.TypeID,vs.TargetCopter,vs.TargetMountType,vs.DataWidth,vs.DataHeight,vs.DataCheckFreq]+vs.SendProtocol
        floValue=[vs.CameraFOV] + vs.SensorPosXYZ+vs.SensorAngEular+vs.otherParams
        buf = struct.pack("16H15f",*intValue,*floValue)
        self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windID))

    def sendReqToUE4(self,windID=0):
        """ send VisSensor list to RflySim3D to request image
        windID specify the index of RflySim3D window to send image
        """
        
        if len(self.VisSensor)<=0:
            print('Error: No sensor is obtained.')
            return False
        
        # EmptyMem = np.zeros(66,dtype=np.int).tolist()
        # buf = struct.pack("66i",*EmptyMem)
        # self.mm0.seek(0)
        # self.mm0.write(buf)
        # self.mm0.seek(0)
        contSeq0=False
        if self.isUE4DirectUDP:
            for i in range(len(self.VisSensor)):
                if self.VisSensor[i].SendProtocol[0]==0: #如果之前设置的是共享内存方式，则强制转化为UDP直发
                    self.VisSensor[i].SendProtocol[0]=1
                if self.RemotSendIP !='' and (self.VisSensor[i].SendProtocol[0]==1 or self.VisSensor[i].SendProtocol[0]==2 or self.VisSensor[i].SendProtocol[0]==3):
                    cList = self.RemotSendIP.split('.')
                    if len(cList)==4:
                        self.VisSensor[i].SendProtocol[1]=int(cList[0])
                        self.VisSensor[i].SendProtocol[2]=int(cList[1])
                        self.VisSensor[i].SendProtocol[3]=int(cList[2])
                        self.VisSensor[i].SendProtocol[4]=int(cList[3])
                if self.VisSensor[i].SeqID==0:
                    contSeq0=True
                    
        if contSeq0:
            self.sendUE4Cmd(b'RflyClearCapture',windID)
        
        for i in range(len(self.VisSensor)):
            # struct VisionSensorReq {
            # 	uint16 checksum; //数据校验位，12345
            # 	uint16 SeqID; //内存序号ID
            # 	uint16 TypeID; //传感器类型ID
            # 	uint16 TargetCopter; //绑定的目标飞机     //可改变
            # 	uint16 TargetMountType; //绑定的类型    //可改变
            # 	uint16 DataWidth;   //数据或图像宽度
            # 	uint16 DataHeight; //数据或图像高度
            # 	uint16 DataCheckFreq; //检查数据更新频率
            # 	uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
            # 	float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
            # 	float SensorPosXYZ[3]; // 传感器安装位置    //可改变
            # 	float SensorAngEular[3]; //传感器安装角度   //可改变
            # 	float otherParams[8]; //预留的八位数据位
            # }16H15f
            vs = self.VisSensor[i]
            intValue=[vs.checksum,vs.SeqID,vs.TypeID,vs.TargetCopter,vs.TargetMountType,vs.DataWidth,vs.DataHeight,vs.DataCheckFreq]+vs.SendProtocol
            floValue=[vs.CameraFOV] + vs.SensorPosXYZ+vs.SensorAngEular+vs.otherParams
            buf = struct.pack("16H15f",*intValue,*floValue)
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windID))
            
        time.sleep(1)
        
        # struct UE4CommMemData {
        # 	int Checksum;//校验位，设置为1234567890
        # 	int totalNum;//最大传感器数量
        # 	int WidthHeigh[64];//分辨率宽高序列，包含最多32个传感器的
        # }
        self.mm0 = mmap.mmap(0, 66*4, 'UE4CommMemData') # 公共区
        Data = np.frombuffer(self.mm0,dtype = np.int)
        checksum = Data[0]
        totalNum = Data[1]
        ckCheck=False
        #print(Data)
        if checksum==1234567890:
            ckCheck=True
            for i in range(len(self.VisSensor)):
                isSucc=False
                vs = self.VisSensor[i]
                idx = vs.SeqID
                width = Data[2+idx*2]
                height = Data[2+idx*2+1]
                if width==vs.DataWidth and height==vs.DataHeight:
                    if idx<=totalNum:
                        isSucc=True
                if not isSucc:
                    ckCheck=False
                    break
        if not ckCheck:
            print('Error: Sensor req failed from UE4.')
            return False
        print('Sensor req success from UE4.')
        self.hasReqUE4=True
        return True


    def img_udp_thrdNew(self,udpSok,idx,typeID):
        CheckSum=1234567890
        fhead_size = struct.calcsize('4i1d')
        imgPackUnit = 60000
        
        seqList=[]
        dataList=[]
        timeList=[]
        recPackNum=0
        timeStmpStore=0
        
        while True:
            buf,addr = udpSok.recvfrom(imgPackUnit+2000) #加一些余量，确保包头数据考虑在内
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
                
                if not math.isclose(timeStmpStore,dd[4],rel_tol=0.00001): #如果时间戳不一致
                    continue #跳过这个包
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
                    #if typeID==2:
                    #    print('Start img cap',self.VisSensor[idx].SendProtocol[0])
                    if self.VisSensor[idx].SendProtocol[0]==1 or self.VisSensor[idx].SendProtocol[0]==3:
                        nparr = np.frombuffer(data_total, np.uint8)
                        colorType=cv2.IMREAD_COLOR
                        if typeID==2:
                            colorType=cv2.IMREAD_ANYDEPTH
                        elif typeID==3:
                            colorType=cv2.IMREAD_GRAYSCALE
                        self.Img[idx] = cv2.imdecode(nparr, colorType)
                        if self.Img[idx] is None:
                            print('Wrong Img decode!')
                            self.hasData[idx] = False
                        else:
                            self.hasData[idx] = True
                            self.timeStmp[idx]=timeStmpStore
                            #print('Img',idx,':',timeStmpStore)
                        
                    elif self.VisSensor[idx].SendProtocol[0]==2:
                        dtyp = np.uint8
                        dim = 3
                        if(typeID==1):
                            dtyp = np.uint8
                            dim = 3
                        elif(typeID==2):
                            dtyp = np.uint16
                            dim = 1
                        elif(typeID==3):
                            dtyp = np.uint8
                            dim = 1
                        DataWidth = self.VisSensor[idx].DataWidth
                        DataHeight = self.VisSensor[idx].DataHeight
                        L = np.frombuffer(data_total, dtype = dtyp)
                        # colorType=cv2.IMREAD_COLOR
                        # if typeID==2 or typeID==3:
                        #     colorType=cv2.IMREAD_GRAYSCALE
                        # self.Img[idx] = cv2.imdecode(nparr, colorType)
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                        self.hasData[idx] = True
                        self.timeStmp[idx]=timeStmpStore
                        #print('Img',idx,':',timeStmpStore)
                    
                    if isEnableRosTrans:
                        pass
                        # 将图片和时间戳，推送到ROS消息中
                

    def img_mem_thrd(self,idxList):
        mmList=[]
        for i in range(len(idxList)):
            idx=idxList[i]
            SeqID = self.VisSensor[idx].SeqID
            DataWidth = self.VisSensor[idx].DataWidth
            DataHeight = self.VisSensor[idx].DataHeight
            typeID=self.VisSensor[idx].TypeID
            dim = 3
            dimSize = 1
            if(typeID==1):
                dim = 3
                dimSize = 1
            elif(typeID==2):
                dim = 2
                dimSize = 1
            elif(typeID==3):
                dim = 1
                dimSize = 1
            mm=mmap.mmap(0, DataWidth*DataHeight*dim*dimSize+1+8, 'RflySim3DImg_'+str(SeqID))
            mmList=mmList+[mm]
        #cv2.IMWRITE_PAM_FORMAT_GRAYSCALE
        while True:
            for kk in range(len(idxList)):
                mm=mmList[kk]
                idx=idxList[kk]
                DataWidth = self.VisSensor[idx].DataWidth
                DataHeight = self.VisSensor[idx].DataHeight
                typeID=self.VisSensor[idx].TypeID
                dtyp = np.uint8
                dim = 3
                if(typeID==1):
                    dtyp = np.uint8
                    dim = 3
                elif(typeID==2):
                    dtyp = np.uint16
                    dim = 1
                elif(typeID==3):
                    dtyp = np.uint8
                    dim = 1
                for ii in range(3): #尝试读取三次内存区域
                    flag = np.frombuffer(mm,dtype = np.uint8,count=1)
                    #print(flag[0])
                    if(flag[0]==2): #图像已写入完成
                        #print(flag[0])
                        mm.seek(0)
                        mm.write_byte(3) # 进入读取状态
                        
                        #开始读取图片
                        #L=np.frombuffer(mm,dtype = np.uint8)
                        self.timeStmp[idx] = np.frombuffer(mm,dtype = np.double,count=1,offset=1)#struct.unpack('d',L[1:9]) #获得时间戳
                        #print('#1:',timeStmp)
                        
                        mm.seek(0)
                        mm.write_byte(4) # 进入读取完成状态
                        
                        L=np.frombuffer(mm,dtype = dtyp,offset=9)
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim) #reshape array to 4 channel image array H X W X 4
                        
                        if self.isRemoteSend:
                            self.sendImgUDPNew(idx)
                        
                        self.hasData[idx] = True
                        #读取到图片后就退出for循环
                        #print("readImg"+str(idx))
                        break
            if self.isRemoteSend:
                time.sleep(0.001)
            else:
                time.sleep(0.005)
            
    def startImgCap(self,isRemoteSend=False):
        """ start loop to receive image from UE4,
        isRemoteSend=true will forward image from memory to UDP port
        """
        self.isRemoteSend = isRemoteSend
        memList=[]
        udpList=[]
        for i in range(len(self.VisSensor)):
            self.Img = self.Img +[0]
            self.hasData = self.hasData +[False]
            self.timeStmp = self.timeStmp + [0]
            IP = str(self.VisSensor[i].SendProtocol[1])+'.'+str(self.VisSensor[i].SendProtocol[2])+'.'+str(self.VisSensor[i].SendProtocol[3])+'.'+str(self.VisSensor[i].SendProtocol[4])
            if IP == '0.0.0.0':
                IP='127.0.0.1'
            if self.RemotSendIP!='':
                IP=self.RemotSendIP
            self.IpList = self.IpList +[IP]
            self.portList = self.portList + [self.VisSensor[i].SendProtocol[5]]
            if self.VisSensor[i].SendProtocol[0] == 0:
                memList = memList + [i]
            else:
                udpList = udpList + [i]
        
        if len(memList)>0:
            self.t_menRec = threading.Thread(target=self.img_mem_thrd, args=(memList,))
            self.t_menRec.start()           
        
        if len(udpList)>0:
            #print('Enter UDP capture')
            for i in range(len(udpList)):
                udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,60000*100)
                udp.bind(('0.0.0.0', self.portList[udpList[i]]))
                typeID=self.VisSensor[udpList[i]].TypeID
                t_udpRec = threading.Thread(target=self.img_udp_thrdNew, args=(udp,udpList[i],typeID,))
                t_udpRec.start()

    def sendImgUDPNew(self,idx):
        img_encode = cv2.imencode('.png', self.Img[idx])[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        
        imgPackUnit = 60000
        imgLen = len(data)
        imgpackNum = imgLen//imgPackUnit+1
        IP = self.IpList[idx]
        if self.RemotSendIP != '':
            IP=self.RemotSendIP
            
        CheckSum=1234567890
        timeStmpSend=self.timeStmp[idx]
        
        #循环发送图片码流        
        for i in range(imgpackNum):
            dataSend=[]
            if imgPackUnit*(i+1)>len(data): # 末尾包数据提取
                dataSend=data[imgPackUnit*i:]
            else:#前面数据包直接提取60000数据
                dataSend=data[imgPackUnit*i:imgPackUnit*(i+1)] 
            PackLen=4*4+8*1+len(dataSend) #fhead的4i1d长度，加上图片数据长度
            fhead = struct.pack('4i1d',CheckSum,PackLen,i,imgpackNum,timeStmpSend) #校验，包长度，包序号，总包数，时间戳
            dataSend=fhead+dataSend # 包头加上图像数据
            self.udp_socket.sendto(dataSend, (IP, self.portList[idx]))# 发送出去

    def jsonLoad(self,ChangeMode=-1,jsonPath=''):
        """ load config.json file to create camera list for image capture,
        if ChangeMode>=0, then the SendProtocol[0] will be set to ChangeMode to change the transfer style
        """
        if len(jsonPath)==0:
            jsonPath=sys.path[0]+'/Config.json'
        if not os.path.exists(jsonPath):
            print("The json file does not exist!")
            return False
        with open(jsonPath,"r",encoding='utf-8') as f:
            jsData=json.loads(f.read())
            if len(jsData["VisionSensors"])<=0:
                print("No sensor data is found!")
                return False                
            for i in range(len(jsData["VisionSensors"])):
                visSenStruct = VisionSensorReq()
                if isinstance(jsData["VisionSensors"][i]["SeqID"],int):
                    visSenStruct.SeqID = jsData["VisionSensors"][i]["SeqID"]
                else:
                    print("Json data format is wrong!")
                    continue
                
                if isinstance(jsData["VisionSensors"][i]["TypeID"],int):
                    visSenStruct.TypeID = jsData["VisionSensors"][i]["TypeID"]
                else:
                    print("Json data format is wrong!")
                    continue
                
                if isinstance(jsData["VisionSensors"][i]["TargetCopter"],int):
                    visSenStruct.TargetCopter = jsData["VisionSensors"][i]["TargetCopter"]
                else:
                    print("Json data format is wrong!")
                    continue
                                
                if isinstance(jsData["VisionSensors"][i]["TargetMountType"],int):
                    visSenStruct.TargetMountType = jsData["VisionSensors"][i]["TargetMountType"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataWidth"],int):
                    visSenStruct.DataWidth = jsData["VisionSensors"][i]["DataWidth"]
                else:
                    print("Json data format is wrong!")
                    continue    
                
                if isinstance(jsData["VisionSensors"][i]["DataHeight"],int):
                    visSenStruct.DataHeight = jsData["VisionSensors"][i]["DataHeight"]
                else:
                    print("Json data format is wrong!")
                    continue                
                
                if isinstance(jsData["VisionSensors"][i]["DataCheckFreq"],int):
                    visSenStruct.DataCheckFreq = jsData["VisionSensors"][i]["DataCheckFreq"]
                else:
                    print("Json data format is wrong!")
                    continue                   

                if isinstance(jsData["VisionSensors"][i]["CameraFOV"],int):
                    visSenStruct.CameraFOV = jsData["VisionSensors"][i]["CameraFOV"]
                else:
                    print("Json data format is wrong!")
                    continue                
                
                
                if len(jsData["VisionSensors"][i]["SendProtocol"])==8:
                    visSenStruct.SendProtocol = jsData["VisionSensors"][i]["SendProtocol"]
                    if ChangeMode!=-1:
                        visSenStruct.SendProtocol[0]=ChangeMode # 如果是远程接收模式，那么读图这里需要配置为UDP接收
                else:
                    print("Json data format is wrong!")
                    continue 

                if len(jsData["VisionSensors"][i]["SensorPosXYZ"])==3:
                    visSenStruct.SensorPosXYZ = jsData["VisionSensors"][i]["SensorPosXYZ"]
                else:
                    print("Json data format is wrong!")
                    continue                 

                if len(jsData["VisionSensors"][i]["SensorAngEular"])==3:
                    visSenStruct.SensorAngEular = jsData["VisionSensors"][i]["SensorAngEular"]
                else:
                    print("Json data format is wrong!")
                    continue 
                
                if len(jsData["VisionSensors"][i]["otherParams"])==8:
                    visSenStruct.otherParams = jsData["VisionSensors"][i]["otherParams"]
                else:
                    print("Json data format is wrong!")
                    continue 
                self.VisSensor=self.VisSensor+[visSenStruct]
        if(len(self.VisSensor))<=0:
            print("No sensor is obtained.")
            return False
        print('Got',len(self.VisSensor),'vision sensors from json')
        return True
