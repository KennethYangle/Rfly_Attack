#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Date    : 2021-05-13 16:15:29
# @Author  : BrightSoul (653538096@qq.com)

import socket
import time

class RClient:
    def __init__(self, HOST='192.168.110.145', PORT=63000):
        self.D_cmd = {
            "strat":b"cmd01",
            "stop":b"cmd02",
            "end":b"cmdff"
        }

        self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.client.connect((HOST,PORT))
        self.client.setblocking(False)

        print("Try to connect Server {} on {}".format(HOST, PORT))

        while True:
            message = b"Hello Server"
            self.client.sendall(message)
            time.sleep(1)
            try:
                data,address = self.client.recvfrom(1024)
            except Exception as e:
                print("no connection")
                continue

            if data == b"Server Received":
                print("Server Received")
                break

    def Start(self):
        self.client.sendall(self.D_cmd["strat"])
        print("Client Start!")
    def Stop(self):
        self.client.sendall(self.D_cmd["stop"])
        print("Client Stop!")
    def End(self):
        self.client.sendall(self.D_cmd["end"])
        print("Client End!")

if __name__=="__main__":
    r = RClient()
    r.Start()
    time.sleep(40)
    r.Stop()
    time.sleep(2)
    r.Start()
    time.sleep(40)
    r.Stop()
    time.sleep(2)
    r.End()