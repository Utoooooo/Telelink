import time
import serial
import math
import numpy as np

print("droneControlFunctions Imported")


def drone_control_init(port='COM6'):
    # Function initiates serial communications with arduino. It selects com port and opens communications
    # Arduino must be running TinyPico_BLE_ToF.ino which turns the commands into PPM and sends to T12 remote.
    # Function returns link to be used when updating
    link = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    return link


def drone_control_update(link, roll=1500, pitch=1500, thrust=1000, yaw=1500):
    #########################################
    # Use pySerial Library
    #########################################
    throttle = ("<%s,%s,%s,%s,>" % (int(thrust), int(yaw), int(roll), int(pitch)))
    print(throttle)
    link.write(bytes(str(throttle), 'utf-8'))
    time.sleep(0.001)


def remap_coord(x_global, y_global, rotation):
    x_local = x_global*math.cos(math.radians(rotation))+y_global*math.sin(math.radians(rotation))
    y_local = -x_global*math.sin(math.radians(rotation)) + y_global*math.cos(math.radians(rotation))
    return x_local, y_local


class droneController:

    def __init__(self, dt, kp=1, ki=0.8, kd=0):
        """Initialise PID controller for drone"""

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.dt = dt

        self.integral = 0
        self.olderror = 0

    def update(self, ref, state):
        """Update step for PID controller"""
        e = ref - state
        # Speed limiter
        if e > 100:
            e = 100
        elif e < -100:
            e = -100
        self.integral = self.integral + e * self.dt
        i = self.integral
        d = e - self.olderror
        self.olderror = e

        actuation = self.kp * e + self.ki * i + self.kd * d

        return actuation


class DistPID:
    def __init__(self, dt, kp=1):
        """Initialise PID controller for drone velocity"""
        self.kp = kp
        self.dt = dt

    def update(self, ref, state):
        """Update step for PID controller"""
        e = ref - state
        # Speed limiter
        if e > 100:
            e = 100
        elif e < -100:
            e = -100

        actuation = self.kp * e

        return actuation


def random_thrust():
    a = np.arange(0, 5000, 1)
    thr_list = 30 * np.sin((a + 5) / 18) + 80 * np.sin((a + 20) / 100) + 6 * np.random.rand(len(a)) + np.ones(len(a)) * 1450 + 30 * np.sin(a / 500)
    # print(b.round())
    return thr_list


def take_off(link, hover_point):
    curr_time = time.time()
    while time.time()-curr_time < 1:
        drone_control_update(link, thrust=1000)
    for i in range(1200, hover_point):
        drone_control_update(link, thrust=i)


def landing(link, hover_point):
    curr_time = time.time()
    for i in range(hover_point, 1000, -2):
        drone_control_update(link, thrust=i)
    while time.time()-curr_time < 1:
        drone_control_update(link, thrust=1000)
