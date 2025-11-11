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

dt = DriveTrain(left_drive, right_drive, 317.0, 320, 320, MM, 1.0)

GYRO_SCALE_FOR_READOUT = 361.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

USING_TRACKING_WHEELS = True

if USING_TRACKING_WHEELS:
    rotation_fwd = Rotation(Ports.PORT6, False)
    rotation_strafe = Rotation(Ports.PORT7, False)
    all_sensors = [inertial, rotation_fwd, rotation_strafe]

    ODOMETRY_FWD_SIZE = 218.344
    ODOMETRY_FWD_OFFSET = 0.375 * 25.4
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 160.0
    ODOMETRY_STRAFE_OFFSET = 4.5 * 25.4
    ODOMETRY_STRAFE_GEAR_RATIO = 1.0

else:
    all_sensors = [inertial]

    ODOMETRY_FWD_SIZE = 317.0
    ODOMETRY_FWD_OFFSET = 0.0
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 0.0

ROBOT_INITIALIZED = False
ROBOT_INITIALIZATION_FAILED = False

def initialize_tracker():
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
    if USING_TRACKING_WHEELS:
        tracker_devices = [rotation_fwd, rotation_strafe, inertial]
    else:
        tracker_devices = [left_drive, right_drive, inertial]

    tracker_thread = Thread(Tracking.tracker_thread, (tracker_configuration, tracker_devices, starting_location))
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
def auton1_drive_straight(drive_train: DriveProxy, tracker: Tracking):
    drive_train.turn_to_heading(90.0, timeout=2.0)
    drive_train.turn_to_heading(0.0, timeout=2.0)

    distance = 36.0 * 25.4
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    drive_train.drive_for(FORWARD, distance, MM, 0.0,timeout=timeout)
    drive_train.drive_for(REVERSE, distance, MM, 0.0, timeout=timeout)
    
    drive_train.turn_to_heading(0.0, timeout=2.0)

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def auton2_drive_to_points(drive_train: DriveProxy, tracker: Tracking):
    print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(90.0, timeout=2.0)
    print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(0.0, timeout=2.0)
    print_tracker(tracker)

    print("Start Drive")
    distance, heading = tracker.trajectory_to_point(36.0 * 25.4, 0.0)
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    print_tracker(tracker, 36.0 * 25.4, 0.0)
    drive_train.turn_to_heading(heading, settle_error=1.0, timeout=0.5)
    drive_train.drive_for(FORWARD, distance, MM, heading, timeout=timeout)
    print_tracker(tracker)

    print("Start Drive")
    distance, heading = tracker.trajectory_to_point(0.0, 0.0)
    heading = InertialWrapper.to_heading(heading + 180.0)
    drive_train.turn_to_heading(heading, settle_error=1.0, timeout=0.5)
    drive_train.drive_for(REVERSE, distance, MM, heading, timeout=timeout)
    print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(0.0)
    print_tracker(tracker)

def auton3_drive_to_points_long(drive_train: DriveProxy, tracker:Tracking):
    print_tracker(tracker)

    #x_near = 0.0
    #x_far = 2.0 * 600.0

    #y_left = 0.0
    #y_right = 1.0 * 600.0

    #points = [
    #    [x_far, y_left],
    #    [x_far, y_right],
    #    [x_near, y_right],
    #    [x_near, y_left]
    #]

    # field tiles are 600mm across (not 18")
    x_near = 0.0
    x_far = 3.0 * 600.0

    y_far_left = -1.5 * 600.0
    y_mid_left = 0.0
    y_mid_right = 2.0 * 600.0
    y_far_right = 3.5 * 600.0

    points = [
        [x_far, y_mid_left],
        [x_far, y_mid_right],
        [x_near, y_mid_right],
        [x_near, y_far_left],
        [x_far, y_far_left],
        [x_far, y_far_right],
        [x_near, y_far_right],
        [x_near, y_mid_left]
    ]

    for point in points:
        print("Start Drive 1", point)
        x = point[0]
        y = point[1]
        distance, heading = tracker.trajectory_to_point(x, y)
        timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
        drive_train.turn_to_heading(heading, settle_error=1.0, timeout=0.5)
        drive_train.drive_for(FORWARD, distance, MM, heading, timeout=timeout)
        print_tracker(tracker)

    print("Start Turn")
    drive_train.turn_to_heading(0.0)
    print_tracker(tracker)


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

    tracker = Tracking()

    # auton1_drive_straight(drive_train, tracker)
    # auton2_drive_to_points(drive_train, tracker)
    auton3_drive_to_points_long(drive_train, tracker)

def user_control():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    print("driver control")

    tracker = Tracking()

    #wait(1, SECONDS)
    #dt.drive_for(FORWARD, 240.0 * 10.0, MM, 25, PERCENT)

    while True:
        print_tracker(tracker)
        wait(1, SECONDS)

comp = Competition(user_control, autonomous)
pre_autonomous()

