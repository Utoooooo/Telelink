"""
Created by Rui Zhou on 19/09/2023
Last Modified by Rui Zhou on 02/10/2023
"""

import socket
import time
import numpy as np
import serial
import struct


class Telelink:
    def __init__(self, local_ip="192.168.1.2", udp_port=4210):
        """

        :param udp_ip: IP address of the board
        :param udp_port: Port number of the board
        """
        self.local_udp_ip = local_ip
        self.shared_udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
        self.sock.bind((self.local_udp_ip, self.shared_udp_port))
        self.tof = 0
        self.acc_z = 0

    def receive(self):
        data, addr = self.sock.recvfrom(16)
        if len(data) == 14:
            acc_x, acc_y, acc_z, roll, pitch, yaw, tof = struct.unpack('hhhhhhh', data)
            print(f"acc_x: {acc_x/512}, acc_y: {acc_y/512}, acc_z: {acc_z/512}, roll: {roll}, pitch: {pitch}, yaw: {yaw}, tof: {tof}")
            self.tof = tof
            self.acc_z = acc_z/512
            # print(f"acc_x: {acc_x}, acc_y: {acc_y}, acc_z: {acc_z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")




class Command:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.timeout = 0.1
        self.link = serial.Serial(port=port, baudrate=115200, timeout=0.1)

    def dc_update(self, roll=1500, pitch=1500, thrust=1000, yaw=1500):
        #########################################
        # Use pySerial Library
        #########################################
        throttle = ("<%s,%s,%s,%s,>" % (int(thrust), int(yaw), int(roll), int(pitch)))
        print(throttle)
        self.link.write(bytes(str(throttle), 'utf-8'))
        time.sleep(0.001)
