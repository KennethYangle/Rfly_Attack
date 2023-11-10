@ECHO OFF

REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Start index of vehicle number (should larger than 0)
REM This option is useful for simulation with multi-computers
SET /a START_INDEX=1


REM Set the start UDP port for SIMULINK/OFFBOARD API
REM This option should not be modified for swarm simulation
SET /a UDP_START_PORT=20100


REM Set use DLL model name or not, use number index or name string
REM This option is useful for simulation with other types of vehicles instead of multicopters
set DLLModel=0

REM Set the simulation mode on CopterSim, use number index or name string
REM e.g., SimMode=0 equals to  SimMode=PX4_HITL
set SimMode=0


REM Set the map, use index or name of the map on CopterSim
REM e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
SET UE4_MAP=VisionRingBlank

REM Set the origin x,y position (m) and yaw angle (degree) at the map
SET /a ORIGIN_POS_X=0
SET /a ORIGIN_POS_Y=0
SET /a ORIGIN_YAW=0

REM Set the interval between two vehicle, unit:m 
SET /a VEHICLE_INTERVAL=2


REM Set broadcast to other computer; 0: only this computer, 1: broadcast; or use IP address to increase speed
REM e.g., IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
SET IS_BROADCAST=1

REM Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
REM e.g., UDPSIMMODE=1 equals to UDPSIMMODE=UDP_Simple
SET UDPSIMMODE=2

REM Set the path of the RflySim tools
SET PSP_PATH=C:\PX4PSP
set CURRENT_PATH=%cd%

%PSP_PATH%\Python38\python.exe _mav_print.py

C:



ECHO.
ECHO ---------------------------------------
setlocal enabledelayedexpansion
for /l %%i in (1,1,3) do set "kong=!kong!#"
for /f "tokens=2 delims=COM" %%i in ('mode ^| find /i "COM"') do (
    for /f "tokens=1 delims=:" %%i in ("%%i") do (
    set "str=%%i%kong%"
    set "a=!str:~,3!"
    set "a=!a:%%i=!"
    set "_!a!%%i=a"
    set HASCOM=1
    )
) 
if not defined HASCOM (
    goto StartInput
)
for /f "delims==_" %%i in ('set _') do (
    set "str=%%i"
    if defined COMDef (
        SET COMDef=!COMDef!,!str:#=!
    ) else (
        SET COMDef=!str:#=!
    )
)

:StartInput
echo Please input the Pixhawk COM port list for HIL
echo Use ',' as the separator if more than one Pixhawk
echo E.g., input 3 for COM3 of Pixhawk on the computer
if defined HASCOM (
    echo Available COM list on this computer is: %COMDef%
) else (
    echo Warning: there is no available COM port
)

ECHO.
ECHO ---------------------------------------
SET /P ComNum=My COM list for HITL simulation is:
SET string=%ComNum%
set subStr = ""
set /a VehicleNum=0
:split
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a eValue=subStr
    if not %eValue% EQU %subStr% (
        echo Error: Input '%subStr%' is not a integer!
        goto EOF
    )
    set /a VehicleNum = VehicleNum +1
if not "%string%"=="" goto split
REM cho total com number is %VehicleNum%

SET /A VehicleTotalNum=%VehicleNum% + %START_INDEX% - 1
if not defined TOTOAL_COPTER (
    SET /A TOTOAL_COPTER=%VehicleTotalNum%
)

set /a sqrtNum=1
set /a squareNum=1
:loopSqrt
set /a squareNum=%sqrtNum% * %sqrtNum%
if %squareNum% EQU %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
if %squareNum% GTR %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
set /a sqrtNum=%sqrtNum%+1
goto loopSqrt
:loopSqrtEnd


REM UE4Path
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
start %PSP_PATH%\RflySim3D\RflySim3D.exe
start %PSP_PATH%\RflySim3D\RflySim3D.exe
start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd %PSP_PATH%\CopterSim

set /a cntr = %START_INDEX%
set /a endNum = %VehicleTotalNum% +1
set /a portNum = %UDP_START_PORT% + ((%START_INDEX%-1)*2)
SET string=%ComNum%
:split1
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a PosXX=((%cntr%-1) / %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_X%
    set /a PosYY=((%cntr%-1) %% %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_Y%
    REM echo start CopterSim
    start /high CopterSim.exe 1 %cntr% %portNum% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% %subStr% %UDPSIMMODE%
    choice /t 1 /d y /n >nul
    set /a cntr=%cntr%+1
    set /a portNum = %portNum% +2
    REM TIMEOUT /T 1
if not "%string%"=="" goto split1

REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe
ECHO Start QGroundControl

choice /t 5 /d y /n >nul
%PSP_PATH%\Python38\python.exe %CURRENT_PATH%\client_ue4.py

pause

REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

ECHO Start End.
