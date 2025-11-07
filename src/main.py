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
from tracker import Tracking

brain=Brain()

l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

all_motors = [l1, l2, r1, r2]

GYRO_SCALE_FOR_READOUT = 361.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

USING_TRACKING_WHEELS = True

if USING_TRACKING_WHEELS:
    rotation_fwd = Rotation(Ports.PORT6, True)
    rotation_strafe = Rotation(Ports.PORT7, False)
    all_sensors = [inertial, rotation_fwd, rotation_strafe]

    ODOMETRY_FWD_SIZE = 260.0
    ODOMETRY_FWD_OFFSET = 0.375 * 25.4
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 220.0
    ODOMETRY_STRAFE_OFFSET = 4.5 * 25.4
    ODOMETRY_STRAFE_GEAR_RATIO = 1.0

else:
    all_sensors = [inertial]

    ODOMETRY_FWD_SIZE = 320.0
    ODOMETRY_FWD_OFFSET = 0.0
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 0.0

ROBOT_INITIALIZED = False
ROBOT_INITIALIZATION_FAILED = False

def initialize_tracker():
    tracker_devices = [left_drive, right_drive, inertial]
    starting_location = Tracking.Orientation(0.0, 0.0, 0.0)
    tracker_configuration = Tracking.Configuration(
            fwd_is_odom=USING_TRACKING_WHEELS,
            fwd_wheel_size=ODOMETRY_FWD_SIZE,
            fwd_gear_ratio=ODOMETRY_FWD_GEAR_RATIO,
            fwd_offset=ODOMETRY_FWD_OFFSET,
            side_wheel_size=ODOMETRY_STRAFE_SIZE,
            side_gear_ratio=ODOMETRY_STRAFE_GEAR_RATIO,
            side_offset=ODOMETRY_STRAFE_OFFSET
        )
    tracker_thread = Thread(Tracking.tracker_thread, (True, tracker_configuration, tracker_devices, starting_location))
    # give tracker some time to get going
    wait(0.1, SECONDS)

def pre_autonomous():
    global ROBOT_INITIALIZED, ROBOT_INITIALIZATION_FAILED

    wait(0.1, SECONDS)

    print("pre_auton")

    for motor in all_motors:
        if not motor.installed(): ROBOT_INITIALIZATION_FAILED = True
    for sensor in all_sensors:
        if not sensor.installed(): ROBOT_INITIALIZATION_FAILED = True

    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    if ROBOT_INITIALIZATION_FAILED:
        brain.screen.clear_screen()
        brain.screen.print("INITIALIZATION FAILED: Check Connections")
        while(True):
            wait(1, SECONDS)

    initialize_tracker()

    ROBOT_INITIALIZED = True

def print_tracker(tracker: Tracking, x = 0.0, y = 0.0):
    orientation = tracker.get_orientation()
    origin_distance, origin_heading = tracker.trajectory_to_point(x, y)
    print("X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(orientation.x, orientation.y, orientation.heading))
    print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))

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

    print("auton")

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

    print("driver control")

    tracker = Tracking()

    while True:
        print_tracker(tracker)
        wait(1, SECONDS)

comp = Competition(user_control, autonomous)
pre_autonomous()

