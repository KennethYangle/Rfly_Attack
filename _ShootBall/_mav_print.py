#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2020-02-02 21:01:46
# @Author  : BrightSoul (653538096@qq.com)


import os
import time
from pymavlink import mavutil
import serial.tools.list_ports



if __name__ == '__main__':
    infoL = []

    port_list = serial.tools.list_ports.comports()
    # print([i.name for i in port_list])
    for i in port_list:
        # print("---"*10)
        the_connection = mavutil.mavlink_connection(i.name,baud=115200)
        # 返回一个HB
        haveHB = the_connection.wait_heartbeat(timeout=2)
        if not haveHB:
            print(f"{i} is not a px4")
            continue

        infoL.append({"com":i.name,"sys_id":the_connection.target_system})

    infoL.sort(key=lambda x:x["sys_id"])

    for i in infoL:
        print("{sys_id} at {com}".format(**i))
    print(["{com}".format(**i) for i in infoL])
    time.sleep(1)
