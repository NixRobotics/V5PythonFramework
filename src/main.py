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
from driveproxy import DriveProxy

brain=Brain()

l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

GYRO_SCALE_FOR_READOUT = 361.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

ROBOT_INITIALIZED = False

def pre_autonomous():
    global ROBOT_INITIALIZED

    wait(0.1, SECONDS)

    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    ROBOT_INITIALIZED = True

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def auton1_drive_straight(drive_train: DriveProxy):
    drive_train.turn_to_heading(90.0, timeout=2.0)
    drive_train.turn_to_heading(0.0, timeout=2.0)

    distance = 36.0 * 25.4
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    drive_train.drive_for(FORWARD, distance, MM, 0.0,timeout=timeout)
    drive_train.drive_for(REVERSE, distance, MM, 0.0, timeout=timeout)
    
    drive_train.turn_to_heading(0.0, timeout=2.0)

def autonomous():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    drive_train = DriveProxy(left_drive, right_drive, inertial)
    drive_train.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0, settle_error=0.5) # degrees
    drive_train.set_drive_constants(Kp=0.5, Ki=0.0, Kd=0.0, settle_error=5) # mm
    drive_train.set_heading_lock_constants(Kp=1.4, Ki=0.0, Kd=0.0, settle_error=0.0) # degrees
    drive_train.set_turn_velocity(66, PERCENT)
    drive_train.set_drive_velocity(66, PERCENT)
    drive_train.set_drive_acceleration(10, PERCENT) # 5% per timestep

    auton1_drive_straight(drive_train)

def user_control():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    while True:
        wait(10, MSEC)

comp = Competition(user_control, autonomous)
pre_autonomous()

