# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Nick                                                         #
# 	Created:      11/6/2025, 8:12:02 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from inertialwrapper import InertialWrapper

GYRO_SCALE_FOR_READOUT = 361.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

# Brain should be defined by default
brain=Brain()

ROBOT_INITIALIZED = False

def pre_autonomous():
    global ROBOT_INITIALIZED

    wait(0.1, SECONDS)

    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    ROBOT_INITIALIZED = True

def autonomous():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

def user_control():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    wait(1, SECONDS)
    print(inertial.rotation())
    inertial.set_heading(270.0)
    print(inertial.rotation())

    while True:
        wait(10, MSEC)

comp = Competition(user_control, autonomous)
pre_autonomous()

