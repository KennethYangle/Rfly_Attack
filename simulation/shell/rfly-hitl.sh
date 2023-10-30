roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.110.145:20100" & PID1=$!
sleep 10s
# roslaunch rflysim_ros_pkg camera.launch & PID2=$!
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch & PID2=$!
sleep 10s
roslaunch simulation rflysim_sphere.launch & PID3=$!
sleep 2s
roslaunch simulation sim_rfly.launch & PID4=$!

# exit
wait
kill -9 PID1 PID2 PID3 PID4 &
exit
