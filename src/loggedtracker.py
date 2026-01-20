# Library imports
from vex import *
from math import sin, cos, radians, degrees, atan2, sqrt
from collections import namedtuple
from array import array
from inertialwrapper import InertialWrapper
from tracker import Tracking
from logger import Logger

class LoggedTracking():
    def __init__(self, motor_tracker: Tracking, motor_devices: List, odom_tracker: Tracking, odom_devices: List):
        self.motor_tracker = motor_tracker
        self.motor_devices = motor_devices
        self.odom_tracker = odom_tracker
        self.odom_devices = odom_devices

        self.track_thread = self.start_thread()

    def read_motor_group(self, mg: MotorGroup):
        timestamp = self.motor_tracker._avg_motor_times(mg)
        return timestamp, mg.position(TURNS)

    def read_rotation(self, r: Rotation):
        return r.timestamp(), r.position(TURNS)
    
    def read_inertial(self, i: InertialWrapper):
        return i.timestamp(), radians(i.rotation(DEGREES))
    
    def tracker_thread(self):
        motor_device_count = len(self.motor_devices)
        odom_device_count = len(self.odom_devices)

        current_motor_timestamps = array('i', [0] * motor_device_count)
        last_motor_timestamps = array('i', [0] * motor_device_count)
        current_motor_values = array('f', [0.0] * motor_device_count)

        current_odom_timestamps = array('i', [0] * odom_device_count)
        last_odom_timestamps = array('i', [0] * odom_device_count)
        current_odom_values = array('f', [0.0] * odom_device_count)

        while True:

            current_motor_timestamps[0], current_motor_values[0] = self.read_motor_group(self.motor_devices[0])
            current_motor_timestamps[1], current_motor_values[1] = self.read_motor_group(self.motor_devices[1])
            current_motor_timestamps[3], current_motor_values[3] = self.read_inertial(self.motor_devices[3])

            current_odom_timestamps[0], current_odom_values[0] = self.read_rotation(self.motor_devices[0])
            current_odom_timestamps[2], current_odom_values[2] = self.read_rotation(self.motor_devices[2])
            current_odom_timestamps[3], current_odom_values[3] = current_motor_timestamps[3], current_motor_values[3]

            updated = False
            for i in range(motor_device_count):
                if current_motor_timestamps[i] != last_motor_timestamps[i]: updated = True

            for i in range(odom_device_count):
                if current_odom_timestamps[i] != last_odom_timestamps[i]: updated = True

            if updated:
                self.motor_tracker._track_step(current_motor_timestamps, current_motor_values)
                self.odom_tracker._track_step(current_odom_timestamps, current_odom_values)

            last_motor_timestamps = current_motor_timestamps
            last_odom_timestamps = current_odom_timestamps

            wait(5, MSEC)

    def start_thread(self):
        return Thread(self.tracker_thread)
    