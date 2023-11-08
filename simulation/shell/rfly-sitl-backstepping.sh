roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.110.145:20100" & PID1=$!
sleep 10s
rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0 & PID2=$!
sleep 2s
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch & PID3=$!
sleep 10s
roslaunch simulation rflysim_sphere.launch & PID4=$!
sleep 2s
roslaunch simulation sim_rfly_backstepping.launch & PID5=$!

# exit
wait
kill -9 PID1 PID2 PID3 PID4 PID5&
exit
