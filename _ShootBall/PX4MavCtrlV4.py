import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
import math
import sys

# PX4 main mode enumeration
class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

# PX4 sub mode enumeration
class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

# define a class for MAVLink initialization
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)



# PX4 MAVLink listen and control API and RflySim3D control API
class PX4MavCtrler:
    """Create a new MAVLink communication instance, 
    For UDP connect
    use format PX4MavCtrler(port,IP), e.g., PX4MavCtrler(20100,'127.0.0.1')
    For hardware connection
    Windows use format PX4MavCtrler('COM3') or PX4MavCtrler('COM3:115200') for Pixhawk USB port connection
    Windows use format 'COM4:57600' for Pixhawk serial port connection
    Linux use format PX4MavCtrler('/dev/ttyUSB0') or PX4MavCtrler('/dev/ttyUSB0:115200') for USB, or '/dev/ttyAMA0:57600' for Serial port (RaspberryPi example)
    """

    # constructor function
    def __init__(self, port=20100,ip='127.0.0.1'):
        self.isInPointMode=False
        self.isCom=False
        self.baud=115200
        if(isinstance(port, str)):
            self.isCom=True
            strlist = port.split(':')
            
            if(len(strlist)>=2):
                if strlist[1].isdigit():
                    self.baud=int(strlist[1])
                port=strlist[0]
                
        self.CopterID=1
        if not self.isCom:
            self.CopterID=int((port-20100)/2)+1
                
        self.f = fifo
        self.stopFlag = False
        self.mav0 = mavlink2.MAVLink(self.f)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
        self.udp_socketUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
        self.udp_socketTrue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUE4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        
        self.ip = ip  # IP address and port number to send data
        self.port = port
        self.uavAngEular = [0, 0, 0]  # Estimated Eular angles from PX4
        self.trueAngEular = [0, 0, 0] # True simulated Eular angles from CopterSim's DLL model
        self.uavAngRate = [0, 0, 0]  # Estimated angular rate from PX4
        self.trueAngRate = [0, 0, 0] # True simulated angular rate from CopterSim's DLL model
        self.uavPosNED = [0, 0, 0] # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.truePosNED = [0, 0, 0] # True simulated position (related to UE4 map center) from CopterSim's DLL model
        self.uavVelNED = [0, 0, 0] # Estimated local velocity from PX4 in NED frame
        self.trueVelNED = [0, 0, 0] # True simulated speed from CopterSim's DLL model  in NED frame
        self.isVehicleCrash=False # is the vehicle crashing with other
        self.isVehicleCrashID=-10 # the vehicle to collide
        self.uavPosGPS = [0, 0, 0] # Estimated GPS position from PX4 in NED frame
        self.uavPosGPSHome = [0, 0, 0] # Estimated GPS home (takeoff) position from PX4 in NED frame
        self.uavGlobalPos = [0, 0, 0] # Estimated global position from PX4 that transferred to UE4 map
        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = False
        self.isArmed = False
        self.hasSendDisableRTLRC = False
        self.UDPMode=2
        self.startTime= time.time()
        self.MaxSpeed=5
        self.stopFlagTrueData=False
        self.hasTrueDataRec=False
        self.isPX4Ekf3DFixed = False
        #print("MAV Instance Created！")

    def sendStartMsg(self,copterID=-1):
        """ send start signals to the network for copters calling waitForStartMsg()
        if copterID=-1, then all copters will start to run
        if copterID>0, then only the copter with specified copterID will start to run
        """
        buf = struct.pack("3i",1234567890,1,copterID)
        """ The struct form is 
        struct startSignal{
            int checksum; // set to 1234567890 to verify the data
            int isStart; // should start to run
            int copterID; // the copter's ID to start
        }
        """
        self.udp_socket.sendto(buf, ('224.0.0.10', 20007)) # multicast address '224.0.0.10' and port 20007 are adopted here
        print("Send start Msg")
        time.sleep(0.03)

    def waitForStartMsg(self):
        """ Program will block until the start signal from sendStartMsg() is received
        """
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
                    """The struct form is 
                        struct startSignal{
                            int checksum; // set to 1234567890 to verify the data
                            int isStart; // should start to run
                            int copterID; // the copter's ID to start
                        }
                    """
                    checksum,isStart,ID=struct.unpack('3i',buf)
                    if checksum==1234567890 and isStart:
                        if ID<0 or ID==self.CopterID:
                            print('Got start Msg, continue to run.')
                            break
            except:
                print("Error to listen to Start Msg!")
                sys.exit(0)


    def initPointMassModel(self,intAlt=0,intState=[0,0,0]):
        """ Init and start the point mass model for UAV control
        intAlt (unit m) is the init height of the vehicle on the UE4 map, which can be obtained from the CopterSim or UE4
        intState contains the PosX (m), PosY (m) and Yaw (degree) of the vehicle, which denotes the initial state of the vehicle
        it is the same as the value on CopterSim UI.
        """
        if self.isCom:
            print('Cannot run Pixhawk in this PointMass Mode!')
            sys.exit(0)
        self.isInPointMode=True
        self.intAlt=intAlt # Init altitude from CopterSim ground height of current map
        self.intStateX=intState[0] # Init PosX of CopterSim
        self.intStateY=intState[1] # Init PosY of CopterSim
        self.intStateYaw=intState[2] # Init Yaw angle of CopterSim
        self.t3 = threading.Thread(target=self.PointMassModelLoop, args=())
        self.t3.start()
        
        
    def EndPointMassModel(self):
        """ End the point mass model
        """
        self.isInPointMode=False
        time.sleep(0.5)
        self.t3.join()

    def yawSat(self,yaw):
        """ satuate the yaw angle from -pi to pi
        """
        if yaw>math.pi:
            yaw = yaw-math.pi*2
            yaw=self.yawSat(yaw)
        elif yaw <-math.pi:
            yaw = yaw+math.pi*2
            yaw=self.yawSat(yaw)
        return yaw

    def PointMassModelLoop(self):
        """ This is the dead loop for point mass model
        """
        # Offboard message sending loop, 100Hz
        self.startTime3 = time.time()
        self.startTime= time.time()
        self.lastTime3 = self.startTime3
        velE=[0,0,0]
        velOff=[0,0,0]
        yawOff=0
        yawRateOff=0
        iNum=0
        while True:
            if not self.isInPointMode:
                break
            self.startTime3 = self.startTime3 + 0.01
            sleepTime = self.startTime3 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.startTime3 = time.time()
            
            dt=time.time()-self.lastTime3
            
            velOff=list(self.vel)
            yawOff=self.yaw
            yawRateOff=self.yawrate
            
            # Calculate desired speed according to target Position
            if self.EnList[1]!=0: # if speed mode
                velOff=list(self.vel)
                yawRateOff=self.yawrate
            elif self.EnList[0]!=0: # if position mode
                targetPosE=list(self.pos)
                if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                    targetPosE[0] = self.pos[0]*math.cos(self.uavAngEular[2])+self.pos[1]*math.sin(self.uavAngEular[2])
                    targetPosE[1] = self.pos[0]*math.sin(self.uavAngEular[2])+self.pos[1]*math.cos(self.uavAngEular[2])
                velOff[0]=self.sat((targetPosE[0]-self.uavPosNED[0])*0.5,self.MaxSpeed)
                velOff[1]=self.sat((targetPosE[1]-self.uavPosNED[1])*0.5,self.MaxSpeed)
                velOff[2]=self.sat((targetPosE[2]-self.uavPosNED[2])*0.5,self.MaxSpeed)
                yawRateOff=self.sat((yawOff-self.uavAngEular[2])*2,math.pi/4)
            else:
                velOff=[0,0,0]
                yawOff=0
                yawRateOff=0
            
            # Calulate vehicle motion according to desired speed
            velE=list(velOff)
            if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                velE[0] = velOff[0]*math.cos(self.uavAngEular[2])+velOff[1]*math.sin(self.uavAngEular[2])
                velE[1] = velOff[0]*math.sin(self.uavAngEular[2])+velOff[1]*math.cos(self.uavAngEular[2])
            
            self.uavVelNED[0]=self.sat(self.uavVelNED[0]*0.97+velE[0]*0.03,15)
            self.uavVelNED[1]=self.sat(self.uavVelNED[1]*0.97+velE[1]*0.03,15)
            self.uavVelNED[2]=self.sat(self.uavVelNED[2]*0.97+velE[2]*0.03,10)
            
            self.uavAngRate[2]=self.sat(self.uavAngRate[2]*0.97+yawRateOff*0.03,math.pi)
            
            # if reach ground
            if self.uavPosNED[2]>0 and velOff[2]>0:
                self.uavVelNED=list([0,0,0])
                self.uavAngRate[2]=0
            
            self.uavPosNED[0]=self.uavVelNED[0]*dt+self.uavPosNED[0]
            self.uavPosNED[1]=self.uavVelNED[1]*dt+self.uavPosNED[1]
            self.uavPosNED[2]=self.uavVelNED[2]*dt+self.uavPosNED[2]
            self.uavAngEular[2]=self.uavAngRate[2]*dt+self.uavAngEular[2]
                
            self.uavAngEular[2]=self.yawSat(self.uavAngEular[2])
            bodyVx = self.uavVelNED[0]*math.cos(-self.uavAngEular[2])+self.uavVelNED[1]*math.sin(-self.uavAngEular[2])
            bodyVy = self.uavVelNED[0]*math.sin(-self.uavAngEular[2])+self.uavVelNED[1]*math.cos(-self.uavAngEular[2])
            
            
            # Calulate desired angle according to speed
            self.uavAngEular[0]=bodyVy/15*math.pi/3
            self.uavAngEular[1]=-bodyVx/15*math.pi/3
            
            
            # calculate vehicle state for UE4
            if self.uavPosNED[2]<-0.01:
                MotorRPMS=[1000,1000,1000,1000,1000,1000,1000,1000]
            else:
                MotorRPMS=[0,0,0,0,0,0,0,0]
            
            self.trueVelNED=list(self.uavVelNED)
            self.trueAngRat=list(self.uavAngRate)
            self.truePosNED[0]=self.intStateX+self.uavPosNED[0]
            self.truePosNED[1]=self.intStateY+self.uavPosNED[1]
            self.truePosNED[2]=self.intAlt+self.uavPosNED[2]
            self.trueAngEular[0]=self.uavAngEular[0]
            self.trueAngEular[1]=self.uavAngEular[1]
            self.trueAngEular[2]=self.yawSat(self.uavAngEular[2]+self.intStateYaw)
            self.uavGlobalPos=list(self.truePosNED)

            sendUE4Msg=True
            if self.CopterID>4:
                sendUE4Msg=False
                iNum=iNum+1
                if iNum%3==0:
                    sendUE4Msg=True
            # if vehicle number<=4, send UE4 msg with 100Hz
            # if vehicle number is too large, send UE4 msg with 33Hz to save network
            if sendUE4Msg:
                # Send vehicle to UE4
                #sendUE4PosSimple(self,copterID,vehicleType,PWMs,VelE,PosE,AngEuler
                self.sendUE4PosSimple(self.CopterID,3,MotorRPMS,self.trueVelNED,self.truePosNED,self.trueAngEular)
            self.lastTime3=time.time()
        #print("Point Mode Stoped.")


    def InitTrueDataLoop(self):
        """ Initialize UDP True data listen loop from CopterSim through 30100 series ports
        """
        self.udp_socketTrue.bind(('0.0.0.0', self.port+1+10000))
        self.stopFlagTrueData=False
        self.tTrue = threading.Thread(target=self.getTrueDataMsg, args=())
        self.tTrue.start()

    def EndTrueDataLoop(self):
        """ End the true data mode
        """
        self.stopFlagTrueData=True
        time.sleep(0.5)
        self.tTrue.join()
        self.hasTrueDataRec=False
        self.udp_socketTrue.close()
        
    def initUE4MsgRec(self):
        """ Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4=False
        MYPORT = 20006
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.udp_socketUE4.bind((ANY,MYPORT))
        status = self.udp_socketUE4.setsockopt(socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start() 
        
    def endUE4MsgRec(self):      
        """ End UE4 message listening
        """  
        self.stopFlagUE4=True
        time.sleep(0.5)
        self.t4.join()
        self.udp_socketUE4.close()
        
    def UE4MsgRecLoop(self):
        """ UE4 message listening dead loop
        """
        lastTime = time.time()
        while True:
            if self.stopFlagUE4:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            
            # struct CopterSimCrash {
            # 	int checksum;
            # 	int CopterID;
            # 	int TargetID;
            # }
            while True:
                if self.stopFlagUE4:
                    break

                try:
                    buf,addr = self.udp_socketUE4.recvfrom(65500)
                    if len(buf)==12:
                        checksum,CopterID,targetID = struct.unpack('iii',buf[0:12])
                        if checksum==1234567890:
                            if targetID>-0.5 and CopterID==self.CopterID:
                                self.isVehicleCrash=True
                                self.isVehicleCrashID=targetID
                            print('Vehicle #',CopterID,' Crashed with vehicle #',targetID)
                except:
                    self.stopFlagUE4=True
                    break
            
        

    def InitMavLoop(self,UDPMode=2):
        """ Initialize MAVLink listen loop from CopterSim
            0 and 1 for UDP_Full and UDP_Simple Modes, 2 and 3 for MAVLink_Full and MAVLink_Simple modes, 4 for MAVLink_NoSend
            The default mode is MAVLink_Full
        """
        self.UDPMode=UDPMode
        if UDPMode>1.5: # UDPMode should lisen to PX4
            if self.isCom:
                self.the_connection = mavutil.mavlink_connection(self.port,self.baud)
            else:
                self.the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))
        else:
            if not self.isCom:
                self.udp_socketUDP.bind(('0.0.0.0', self.port+1))
        self.lastTime = 0
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1.start()
        self.t2 = threading.Thread(target=self.OffboardSendMode, args=())
        self.startTime = time.time()
        self.lastTime2 = 0
        self.startTime2 = time.time()

    def endMavLoop(self):
        """ The same as stopRun(), stop message listenning from 20100 or serial port
        """
        self.stopRun()

    # saturation function
    def sat(self,inPwm=0,thres=1):
        """Saturation function for value inPwm with range thres
        if inPwm>thres, then inPwm=thres
        if inPwm<-thres,then inPwm=-thres
        """
        outPwm= inPwm
        if inPwm>thres:
            outPwm = thres
        elif inPwm<-thres:
            outPwm = -thres
        return outPwm

    # send MAVLink command long message to Pixhawk (routed through CopterSim)
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """ Send command long message to PX4, the mavlink command definitions can be found at
        https://mavlink.io/en/messages/common.html#COMMAND_LONG
        https://mavlink.io/en/messages/common.html#MAV_CMD
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                            command, 0,
                                            param1, param2, param3, param4, param5, param6, param7)
        else: 
            buf = self.mav0.command_long_encode(self.the_connection.target_system, self.the_connection.target_component,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send command to make Pixhawk enter Offboard mode
    def sendMavOffboardCmd(self,type_mask,coordinate_frame, x,  y,  z,  vx,  vy,  vz,  afx,  afy,  afz,  yaw, yaw_rate):
        """ send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.set_position_target_local_ned_send(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate)
        else:
            buf = self.mav0.set_position_target_local_ned_encode(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    def TypeMask(self,EnList):
        """ Obtain the bitmap for offboard message
        https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
        """
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
        type_mask = y
        return int(type_mask)        

    # set the control sigals for Offboard sending loop
    def sendMavOffboardAPI(self,type_mask=0,coordinate_frame=0,pos=[0,0,0],vel=[0,0,0],acc=[0,0,0],yaw=0,yawrate=0):
        """send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        if self.isInPointMode:
            return
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isCom:
            self.the_connection.mav.set_position_target_local_ned_send(int(time_boot_ms),self.the_connection.target_system,
                                                                self.the_connection.target_component,
                                                                coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                vel[0],  vel[1],  vel[2],  acc[0],
                                                                acc[1],  acc[2],  yaw, yawrate)
        else:
            if self.UDPMode>1.5:
                buf = self.mav0.set_position_target_local_ned_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                    vel[0],  vel[1],  vel[2],  acc[0],
                                                                    acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
                self.udp_socket.sendto(buf, (self.ip, self.port))
            else:
                # UDP_Full Mode
                if self.UDPMode==0:
                    # struct inHILCMDData{
                    #     uint32_t time_boot_ms;
                    #     uint32_t copterID;
                    #     uint32_t modes;
                    #     uint32_t flags;
                    #     float ctrls[16];
                    # };
                    # typedef struct _netDataShortShort {
                    #     TargetType tg;
                    #     int        len;
                    #     char       payload[PAYLOAD_LEN_SHORT_SHORT];
                    # }netDataShortShort;
                    ctrls=pos+vel+acc+[yaw,yawrate]+[0,0,0,0,0]
                    buf0 = struct.pack("4I16f",time_boot_ms,self.CopterID,type_mask,coordinate_frame,*ctrls)
                    # buf for remaining 192-152=40bytes of payload[192] of netDataShort
                    buf1 = bytes([0]*(112-len(buf0)))
                    # buf for tg and len in netDataShort
                    buf2 = struct.pack("2i",3,len(buf0))
                    # buf for netDataShort
                    buf=buf2+buf0+buf1
                    self.udp_socket.sendto(buf, (self.ip, self.port))
                else: # UDP_Simple Mode
                    
                    # struct inOffboardShortData{
                    #     int checksum;
                    #     int ctrlMode;
                    #     float controls[4];
                    # };
                    
                    checksum=1234567890
                    ctrlMode=0
                    ctrls=vel+[yawrate]
                    
                    if self.EnList[0]==1 and coordinate_frame == 8: # POS, MAV_FRAME_BODY_NED
                        ctrlMode=3
                        ctrls=pos+[yaw]
                    elif self.EnList[0]==0 and coordinate_frame == 8: # Vel, MAV_FRAME_BODY_NED
                        ctrlMode=1
                        ctrls=vel+[yawrate]
                        #print(ctrlMode,ctrls)
                    elif self.EnList[0]==1 and coordinate_frame == 1: # POS, MAV_FRAME_LOCAL_NED
                        ctrlMode=2
                        ctrls=pos+[yaw]
                    else: # Vel, MAV_FRAME_LOCAL_NED
                        ctrlMode=0
                        ctrls=vel+[yawrate]
                    buf = struct.pack("2i4f",checksum,ctrlMode,ctrls[0],ctrls[1],ctrls[2],ctrls[3])
                    self.udp_socket.sendto(buf, (self.ip, self.port))
                    #print(ctrlMode,ctrls)


    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNED(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame with yawrate (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNEDNoYaw(self,vx,vy,vz):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame without yaw control
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0


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

    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelFRD(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame with yawrate control (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate


    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelNoYaw(self,vx,vy,vz):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame without yawrate control (rad)
        when the vehicle fly upward, the vz < 0
        """
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0

    # send target position in earth NED frame
    def SendPosNED(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in earth NED frame
    def SendPosNEDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    # send target position in body FRD frame
    def SendPosFRD(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in body FRD frame
    def SendPosFRDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    def SendPosNEDExt(self,x=0,y=0,z=0,mode=3,isNED=True):
        """ Send vehicle targe position (m) to PX4 
        when the vehicle fly above the ground, then z < 0
        """
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        if mode==0:
            # Gliding setpoint
            self.type_mask=int(292) # only for fixed Wing
        elif mode==1:
            # Takeoff setpoints
            self.type_mask=int(4096) # only for fixed Wing
        elif mode==2:
            # Land setpoints
            self.type_mask=int(8192) # only for fixed Wing
        elif mode==3:
            # Loiter setpoints
            # for Rover:  Loiter setpoint (vehicle stops when close enough to setpoint).
            # for fixed wing:  Loiter setpoint (fly a circle centred on setpoint).
            self.type_mask=int(12288)    
        elif mode==4:
            # Idle setpoint 
            # only for fixed wing
            # Idle setpoint (zero throttle, zero roll / pitch).
            self.type_mask=int(16384)   
        if isNED:
            self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        else:
            self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0       
        
    def SendCruiseSpeed(self,Speed=0): 
        """ Send command to change the Cruise speed (m/s) of the aircraft
        """
        #def SendCruiseSpeed(self,Speed,Type=1,Throttle=-1,Relative=0): 
        #  type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); min:0 max:3 increment:1
        #  Speed (-1 indicates no change); min: -1; Unit: m/s
        #  Throttle (-1 indicates no change); min: -1; Unit: %
        #  Relative	0: absolute, 1: relative; min:0 max:1 increment:1
        #self.SendMavCmdLong(mavlink2.MAV_CMD_DO_CHANGE_SPEED, Type,Speed,Throttle,Relative,0,0,0)
        #self.sendMavSetParam('NAV_LOITER_RAD'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.sendMavSetParam('FW_AIRSPD_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)

    def SendCopterSpeed(self,Speed=0): 
        """ send command to set the maximum speed of the multicopter
        """
        # 最小3，最大20，默认5
        self.sendMavSetParam('MPC_XY_VEL_MAX'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    def SendGroundSpeed(self,Speed=0): 
        """ Send command to change the ground speed (m/s) of the aircraft
        """
        self.sendMavSetParam('GND_SPEED_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    def SendCruiseRadius(self,rad=0): 
        """ Send command to change the Cruise Radius (m) of the aircraft
        """
        self.sendMavSetParam('NAV_LOITER_RAD'.encode(), rad, mavlink2.MAV_PARAM_TYPE_REAL32)


    def sendTakeoffMode(self):
        """ Send command to make the aircraft takeoff
        """
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)

    # initialize Offboard in Pixhawk and start sending data loop in Python
    def initOffboard(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        
        # if not self.isPX4Ekf3DFixed:
        #     print('CopterSim still not 3DFxied, please wait and try again.');
        #     sys.exit(0)
        
        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = True
        if self.UDPMode>1.5:
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            time.sleep(0.5)
            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.5)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        else:
            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)

    def initOffboard2(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        self.isInOffboard = True
        self.t2.start()
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        if self.UDPMode>1.5:
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)  
            time.sleep(0.5)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)      

    def sendMavTakeOff(self,xM=0,yM=0,zM=0):
        """ Send command to make aircraft takeoff to the desired local position (m)
        """
        pitchRad=15.0/180*math.pi
        yawRad=0
        lat=self.uavPosGPSHome[0]+xM/6381372/math.pi*180
        lon=self.uavPosGPSHome[1]+yM/6381372/math.pi*180
        alt=self.uavPosGPSHome[2]-zM
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchRad/math.pi*180,0,0,yawRad/math.pi*180,lat,lon,alt)

    def sendMavTakeOffGPS(self,lat,lon,alt):
        """ Send command to make aircraft takeoff to the desired global position (degree)
        """
        pitchRad=15.0/180*math.pi
        yawRad=0
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchRad/math.pi*180,0,0,yawRad/math.pi*180,lat,lon,alt)

    def sendMavLand(self,xM,yM,zM):
        """ Send command to make aircraft land to the desired local position (m)
        """
        yawRad=0
        lat=self.uavPosGPSHome[0]+xM/6381372/math.pi*180
        lon=self.uavPosGPSHome[1]+yM/6381372/math.pi*180
        alt=self.uavPosGPSHome[2]-zM
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)
    
    def sendMavLandGPS(self,lat,lon,alt):
        """ Send command to make aircraft land to the desired global position (degree)
        """
        yawRad=0
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)


    # stop Offboard mode
    def endOffboard(self):
        """ Send command to px4 to out offboard mode, and stop the message sending loop
        """
        self.isInOffboard = False
        if self.UDPMode>1.5 and self.hasSendDisableRTLRC:
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, False)
            self.hasSendDisableRTLRC = False
        self.t2.join()

    # send command pixhawk to modify its parameters
    def sendMavSetParam(self,param_id, param_value, param_type):
        """ Send command to px4 to change desired parameter
        the following mavlink message is adopted
        https://mavlink.io/en/messages/common.html#PARAM_SET
        the parameter list can be found in QGC
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.param_set_send(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type)
        else:            
            buf = self.mav0.param_set_encode(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send hil_actuator_controls message to Pixhawk (for rfly_ctrl uORB message)
    def SendHILCtrlMsg(self,ctrls):
        """ Send hil_actuator_controls command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        controls = [1500,1500,1100,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        for i in range(len(ctrls)):
            if i<len(controls):
                controls[i]=ctrls[i]
        if self.isCom:
            self.the_connection.mav.hil_actuator_controls_send(time_boot_ms,controls,1,1)
        else:
            buf = self.mav0.hil_actuator_controls_encode(time_boot_ms,controls,1,1).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))
        #print("Msg Send.")

    # send debug_vect message to Pixhawk to update rfly_ctrl uORB message
    def SendHILCtrlMsg1(self):
        """  Send debug_vect command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#DEBUG_VECT
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        name = b'hello'
        if self.isCom:
            self.the_connection.mav.debug_vect_send(name, time_boot_ms, 1100, 1500, 1700)
        else:
            buf = self.mav0.debug_vect_encode(name, time_boot_ms, 1100, 1500, 1700).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))
        #print("Msg1 Send.")

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    def SendMavArm(self, isArm=0):
        """ Send command to PX4 to arm or disarm the drone
        """
        if (isArm):
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        else:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)

    # send MAVLink rc_channels_override message to override the RC signals
    def SendRcOverride(self, ch1=1500, ch2=1500, ch3=1100, ch4=1500, ch5=1100, ch6=1100, ch7=1500, ch8=1500):
        """ Send MAVLink command to PX4 to override the RC signal 
        ch1~ch8 range from 1000 to 2000
        https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        """
        if self.isCom:
            self.the_connection.mav.rc_channels_override_send(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8)
        else:
            buf = self.mav0.rc_channels_override_encode(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink message manual_control to send normalized and calibrated RC sigals to pixhawk
    def sendMavManualCtrl(self, x=0,y=0,z=0,r=0):
        """ Send MAVLink command to PX4 to override the manual control signal 
        x,y,z,r range from -1000 to 1000
        https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom:
            self.the_connection.mav.manual_control_encode(self.the_connection.target_system, x,y,z,r,0)
        else:
            buf = self.mav0.manual_control_encode(self.the_connection.target_system, x,y,z,r,0).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink command to change current flight mode
    def SendSetMode(self,mainmode,cusmode=0):
        """ Send MAVLink command to PX4 to change flight mode
        https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
        """
        basemode = mavlink2.MAV_MODE_FLAG_HIL_ENABLED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, basemode, mainmode, cusmode)

    # Stop MAVLink data listening loop
    def stopRun(self):
        """ stop mavlink listening loop for InitMavLoop(), the same as endMavLoop()
        """
        if self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)        
        self.stopFlag=True
        time.sleep(0.5)
        self.t1.join()
        if(self.isInOffboard):
            self.endOffboard()
        if self.UDPMode>1.5:
            self.the_connection.close()
        else:
            if not self.isCom:
                self.udp_socketUDP.close()

    # Update Pixhawk states from MAVLink for 100Hz
    def getTrueDataMsg(self):
        """ Start loop to listen True data from 30100 serial data
        """
        lastTime = time.time()
        while True:
            if self.stopFlagTrueData:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            
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
            while True:
                if self.stopFlagTrueData:
                    break

                try:
                    buf,addr = self.udp_socketTrue.recvfrom(65500)
                    if len(buf)==192+8:
                        #print(len(buf[0:8]))
                        tg,strLen = struct.unpack('ii',buf[0:8])
                        if strLen==152:
                            UIV=struct.unpack('2i1d27f3d',buf[8:8+152])
                            #print(self.uavGlobalPos[2])
                            self.trueAngEular[0]=UIV[9]
                            self.trueAngEular[1]=UIV[10]
                            self.trueAngEular[2]=UIV[11]
                            self.truePosNED[0]=UIV[6]
                            self.truePosNED[1]=UIV[7]
                            self.truePosNED[2]=UIV[8]
                            self.trueVelNED[0]=UIV[3]
                            self.trueVelNED[1]=UIV[4]
                            self.trueVelNED[2]=UIV[5]
                            self.trueAngRate[0]=UIV[27]
                            self.trueAngRate[1]=UIV[28]
                            self.trueAngRate[2]=UIV[29]
                            if not self.hasTrueDataRec:
                                self.hasTrueDataRec=True
                        if tg>-0.5:
                            self.isVehicleCrash=True
                            self.isVehicleCrashID=tg
                        
                    if len(buf)==12:
                        checksum,CopterID,targetID = struct.unpack('iii',buf[0:12])
                        if checksum==1234567890:
                            if targetID>-0.5:
                                self.isVehicleCrash=True
                                self.isVehicleCrashID=targetID
                except:
                    self.stopFlagTrueData=True
                    break
            
            
    # Update Pixhawk states from MAVLink for 100Hz
    def getMavMsg(self):
        """ Start loop to listen mavlink data from 20100 series port or COM port
        """
        lastTime = time.time()
        while True:
            if self.stopFlag:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())
            while True:
                if self.stopFlag:
                    break
                
                if self.UDPMode>1.5: # If Use MAVLink Mode
                    msg = self.the_connection.recv_match(
                        type=['ATTITUDE', 'LOCAL_POSITION_NED','HEARTBEAT','HOME_POSITION','GLOBAL_POSITION_INT','HIL_ACTUATOR_CONTROLS'],
                        blocking=False)
                    if msg is not None:
                        if msg.get_type() == "ATTITUDE":
                            self.uavAngEular[0] = msg.roll
                            self.uavAngEular[1] = msg.pitch
                            self.uavAngEular[2] = msg.yaw
                            self.uavAngRate[0] = msg.rollspeed
                            self.uavAngRate[1] = msg.pitchspeed
                            self.uavAngRate[2] = msg.yawspeed
                        if msg.get_type() == "LOCAL_POSITION_NED":
                            self.uavPosNED[0] = msg.x
                            self.uavPosNED[1] = msg.y
                            self.uavPosNED[2] = msg.z
                            self.uavVelNED[0] = msg.vx
                            self.uavVelNED[1] = msg.vy
                            self.uavVelNED[2] = msg.vz
                            if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-40.1540302)/180*math.pi*6362000+self.uavPosNED[0]
                                self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-116.2593683)/180*math.pi*4.8823e6+self.uavPosNED[1]
                                self.uavGlobalPos[2]=-self.uavPosGPSHome[2]+self.uavPosNED[2]

                        if msg.get_type() == "HOME_POSITION":
                            self.uavPosGPSHome[0] = msg.latitude/1e7
                            self.uavPosGPSHome[1] = msg.longitude/1e7
                            self.uavPosGPSHome[2] = msg.altitude/1e3

                        if msg.get_type() == "GLOBAL_POSITION_INT":
                            self.uavPosGPS[0] = msg.lat/1e7
                            self.uavPosGPS[1] = msg.lon/1e7
                            self.uavPosGPS[2] = msg.alt/1e3

                        if msg.get_type() == "HEARTBEAT":
                            isArmed = msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
                            if not self.isArmed and isArmed:
                                print("PX4 Armed!")
                            if self.isArmed and not isArmed:
                                print("PX4 DisArmed!")
                            self.isArmed = isArmed
                            #print("HeartBeat!")

                        if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                            if msg.flags == 1234567890:
                                if not self.hasTrueDataRec:
                                    if msg.mode == 2 and msg.controls[15]>-0.5:
                                        self.isVehicleCrash = True
                                        self.isVehicleCrashID=int(msg.controls[15])
                                    self.trueAngEular[0]=msg.controls[0]
                                    self.trueAngEular[1]=msg.controls[1]
                                    self.trueAngEular[2]=msg.controls[2]
                                    self.truePosNED[0]=msg.controls[3]
                                    self.truePosNED[1]=msg.controls[4]
                                    self.truePosNED[2]=msg.controls[5]
                                    self.trueVelNED[0]=msg.controls[6]
                                    self.trueVelNED[1]=msg.controls[7]
                                    self.trueVelNED[2]=msg.controls[8]
                                    self.trueAngRate[0]=msg.controls[9]
                                    self.trueAngRate[1]=msg.controls[10]
                                    self.trueAngRate[2]=msg.controls[11]
                                if not self.isPX4Ekf3DFixed and msg.mode != 2 and msg.controls[15]>0.5:
                                    self.isPX4Ekf3DFixed=True
                    else:
                        break
                else: 
                    if self.UDPMode==0:
                        # if use UDP Mode
                        #II3i3i3iiiiii3f3f3ffff
                        # struct outHILStateData{ // mavlink data forward from Pixhawk
                        #     uint32_t time_boot_ms; //Timestamp of the message
                        #     uint32_t copterID;     //Copter ID start from 1
                        #     int32_t GpsPos[3];     //Estimated GPS position，lat&long: deg*1e7, alt: m*1e3 and up is positive
                        #     int32_t GpsVel[3];     //Estimated GPS velocity, NED, m/s*1e2->cm/s
                        #     int32_t gpsHome[3];     //Home GPS position, lat&long: deg*1e7, alt: m*1e3 and up is positive
                        #     int32_t relative_alt;  //alt: m*1e3 and up is positive
                        #     int32_t hdg;           //Course angle, NED,deg*1000, 0~360
                        #     int32_t satellites_visible; //GPS Raw data, sum of satellite
                        #     int32_t fix_type;     //GPS Raw data, Fixed type, 3 for fixed (good precision)
                        #     int32_t resrveInit;       //Int, reserve for the future use
                        #     float AngEular[3];    //Estimated Euler angle, unit: rad/s
                        #     float localPos[3];    //Estimated locoal position, NED, unit: m
                        #     float localVel[3];    //Estimated locoal velocity, NED, unit: m/s
                        #     float pos_horiz_accuracy;   //GPS horizontal accuracy, unit: m
                        #     float pos_vert_accuracy; //GPS vertical accuracy, unit: m
                        #     float resrveFloat;      //float,reserve for the future use
                        # }
                        # typedef struct _netDataShortShort {
                        #     TargetType tg;
                        #     int        len;
                        #     char       payload[112];
                        # }netDataShortShort;
                        try:
                            buf,addr = self.udp_socketUDP.recvfrom(65500)
                            if len(buf)==112+8:
                                #print(len(buf[0:8]))
                                tg,strLen = struct.unpack('ii',buf[0:8])
                                if strLen==112:
                                    UIV=struct.unpack('2I14i12f',buf[8:120])
                                    #GpsPos[0]=
                                    #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                                    for idx in range(3):
                                        self.uavAngEular[idx]=UIV[16+idx]
                                        self.uavPosNED[idx]=UIV[19+idx]
                                        self.uavVelNED[idx]=UIV[22+idx]
                                    self.uavPosGPS[0] = UIV[2]/1e7
                                    self.uavPosGPS[1] = UIV[3]/1e7
                                    self.uavPosGPS[2] = UIV[4]/1e3
                                    self.uavPosGPSHome[0] = UIV[8]/1e7
                                    self.uavPosGPSHome[1] = UIV[9]/1e7
                                    self.uavPosGPSHome[2] = UIV[10]/1e3
                                    if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                        self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-40.1540302)/180*math.pi*6362000+self.uavPosNED[0]
                                        self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-116.2593683)/180*math.pi*4.8823e6+self.uavPosNED[1]
                                        self.uavGlobalPos[2]=-(self.uavPosGPSHome[2]-0)+self.uavPosNED[2]
                                    if not self.isPX4Ekf3DFixed and UIV[27]>0.5:
                                        self.isPX4Ekf3DFixed=True
                                    #print(self.uavGlobalPos[2])
                                    if tg>-0.5:
                                        self.isVehicleCrash=True
                                        self.isVehicleCrashID=tg
                                    
                        except:
                            self.stopFlag=True
                            break
                    else:
                        try:
                            buf,addr = self.udp_socketUDP.recvfrom(65500)
                            if len(buf)==52:
                                #print(len(buf[0:8]))
                                UIV=struct.unpack('4i9f',buf)
                                checksum=UIV[0]
                                #GpsPos[0]=
                                #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                                if checksum==1234567890:
                                    for idx in range(3):
                                        self.uavAngEular[idx]=UIV[4+idx]
                                        self.uavPosNED[idx]=UIV[7+idx]
                                        self.uavVelNED[idx]=UIV[10+idx]
                                    self.uavPosGPSHome[0] = UIV[1]/1e7
                                    self.uavPosGPSHome[1] = UIV[2]/1e7
                                    self.uavPosGPSHome[2] = UIV[3]/1e3
                                    if not (abs(self.uavPosGPSHome[0])<1 and abs(self.uavPosGPSHome[1])<1):
                                        self.uavGlobalPos[0]=(self.uavPosGPSHome[0]-40.1540302)/180*math.pi*6362000+self.uavPosNED[0]
                                        self.uavGlobalPos[1]=(self.uavPosGPSHome[1]-116.2593683)/180*math.pi*4.8823e6+self.uavPosNED[1]
                                        self.uavGlobalPos[2]=-(self.uavPosGPSHome[2]-0)+self.uavPosNED[2]
                                    #print(self.uavGlobalPos[2])
                                elif checksum==1234567891:
                                    if not self.isPX4Ekf3DFixed:
                                        self.isPX4Ekf3DFixed=True
                                elif checksum==1234567892 and UIV[2]>-0.5:
                                    self.isVehicleCrash=True
                                    self.isVehicleCrashID=UIV[2]
                                    
                        except:
                            self.stopFlag=True
                            break
        #print("Mavlink Stoped.")

    # Offboard message sending loop, 100Hz
    def OffboardSendMode(self):
        lastTime2 = time.time()
        
        interTime=0.01
        # If in Simple mode, we use 30Hz to reduce network load 
        if self.UDPMode==1 or self.UDPMode==3 or self.UDPMode==4:
            interTime=1/30.0
        
        while True:
            if not self.isInOffboard:
                break
            lastTime2 = lastTime2 + interTime
            sleepTime = lastTime2 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime2 = time.time()
            if self.isInOffboard:
                self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
        #print("Offboard Stoped.")



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