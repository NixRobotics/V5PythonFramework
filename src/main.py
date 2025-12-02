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
import gc
from math import pi, degrees, radians
from inertialwrapper import InertialWrapper
from driveproxy import DriveProxy
from tracker import Tracking
from smartdrvwrapper import SmartDriveWrapper

brain=Brain()
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)
DRIVETRAIN_WHEEL_SIZE = 316.0
DRIVETRAIN_GEAR_RATIO = 1.0

all_motors = [l1, l2, r1, r2]

GYRO_SCALE_FOR_READOUT = 362.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

USING_TRACKING_WHEELS = True
USING_RESAMPLING = True

if USING_TRACKING_WHEELS:
    rotation_fwd = Rotation(Ports.PORT6, False)
    rotation_strafe = Rotation(Ports.PORT7, False)
    all_sensors = [inertial, rotation_fwd, rotation_strafe]

    ODOMETRY_FWD_SIZE = 218.344
    ODOMETRY_FWD_OFFSET = 0.0316 * 25.4
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 157.38
    ODOMETRY_STRAFE_OFFSET = 4.526 * 25.4
    ODOMETRY_STRAFE_GEAR_RATIO = 1.0

else:
    all_sensors = [inertial]

    ODOMETRY_FWD_SIZE = DRIVETRAIN_WHEEL_SIZE
    ODOMETRY_FWD_OFFSET = 0.0
    ODOMETRY_FWD_GEAR_RATIO = DRIVETRAIN_GEAR_RATIO
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 0.0

ROBOT_INITIALIZED = False
ROBOT_INITIALIZATION_FAILED = False

tracker = None  # type: Tracking | None

def initialize_tracker():
    global tracker
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

    tracker = Tracking(tracker_devices, starting_location, tracker_configuration, initial_values=None)
    tracker.enable_resampling(USING_RESAMPLING)
    # give tracker some time to get going
    wait(0.1, SECONDS)

def pre_autonomous():
    global ROBOT_INITIALIZED, ROBOT_INITIALIZATION_FAILED

    wait(0.1, SECONDS)

    brain.screen.clear_screen()
    brain.screen.print("pre auton")
    brain.screen.new_line()
    print("pre_auton")

    for motor in all_motors:
        if not motor.installed(): ROBOT_INITIALIZATION_FAILED = True
    for sensor in all_sensors:
        if not sensor.installed(): ROBOT_INITIALIZATION_FAILED = True

    print("calibratiing ...")
    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    if ROBOT_INITIALIZATION_FAILED:
        brain.screen.clear_screen()
        brain.screen.print("INITIALIZATION FAILED: Check Connections")
        while(True):
            wait(1, SECONDS)
    print("done ...")

    initialize_tracker()

    print("pre_auton done")

    ROBOT_INITIALIZED = True

def print_tracker(tracker: Tracking, x = 0.0, y = 0.0):
    orientation = tracker.get_orientation()
    origin_distance, origin_heading = tracker.trajectory_to_point(x, y)
    print("X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(orientation.x, orientation.y, orientation.heading))
    print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))
    #print(" - Perf: {:.3f}us, {},{},{},{}".format(
    #    tracker.avg_time,
    #    tracker.previous_timestamps.left,
    #    tracker.previous_timestamps.right,
    #    tracker.previous_timestamps.side,
    #    tracker.previous_timestamps.theta))

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
    print("auton3_drive_to_points_long")
    print_tracker(tracker)

    '''
    x_near = 0.0
    x_far = 2.0 * 600.0

    y_left = 0.0
    y_right = 1.0 * 600.0

    points = [
        [x_far, y_left],
        [x_far, y_right],
        [x_near, y_right],
        [x_near, y_left]
    ]
    '''

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

    controller1 = Controller()

    for i in range(4):
        for point in points:
            print("")
            print("----- Start Drive", i, point)
            x = point[0]
            y = point[1]
            print_tracker(tracker, x, y)
            print(inertial.rotation())
            #while not controller1.buttonA.pressing():
            #    wait(10, MSEC)
            distance, heading = tracker.trajectory_to_point(x, y)
            timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
            drive_train.turn_to_heading(heading, settle_error=1.0, timeout=1.0)
            print_tracker(tracker, x, y)
            #while not controller1.buttonA.pressing():
            #    wait(10, MSEC)
            distance, heading = tracker.trajectory_to_point(x, y)
            drive_train.drive_for(FORWARD, distance, MM, heading, timeout=timeout)
            wait(0.1, SECONDS)
            print_tracker(tracker, x, y)

    print("Start Turn")
    drive_train.turn_to_heading(0.0, settle_error=0.25, timeout=2.0)
    wait(0.1, SECONDS)
    print_tracker(tracker, x_near, y_mid_left)

def OnTimeout():
    pass

def calibration_tracking_wheels():

    dt = SmartDriveWrapper(left_drive, right_drive, inertial, DRIVETRAIN_WHEEL_SIZE, 320, 320, MM, 1.0)
    dt.on_timeout(OnTimeout)

    wait(1, SECONDS)

    fwd_start = rotation_fwd.position(RotationUnits.REV)
    side_start = rotation_strafe.position(RotationUnits.REV)
    dt.drive_for(FORWARD, 3.0 * 600.0, MM, 25, PERCENT)
    wait(0.5, SECONDS)
    fwd_end = rotation_fwd.position(RotationUnits.REV)
    side_end = rotation_strafe.position(RotationUnits.REV)
    print(fwd_start, ",", side_start, ",", fwd_end, ",", side_end)

    wait(1, SECONDS)

    fwd_start = rotation_fwd.position(RotationUnits.REV)
    side_start = rotation_strafe.position(RotationUnits.REV)
    dt.drive_for(REVERSE, 3.0 * 600.0, MM, 25, PERCENT)
    wait(0.5, SECONDS)
    fwd_end = rotation_fwd.position(RotationUnits.REV)
    side_end = rotation_strafe.position(RotationUnits.REV)
    print(fwd_start, ",", side_start, ",", fwd_end, ",", side_end)

    wait(1, SECONDS)

    dt.set_turn_constant(0.7)

    gyro_start = inertial.rotation()
    smart_start = dt.rotation()
    print(gyro_start, smart_start)

    fwd_start = rotation_fwd.position(RotationUnits.REV)
    side_start = rotation_strafe.position(RotationUnits.REV)
    dt.turn_for(RIGHT, 180.0, DEGREES, 33, PERCENT, True)
    dt.stop(COAST)
    wait(0.5, SECONDS)
    gyro_end = inertial.rotation()
    fwd_end = rotation_fwd.position(RotationUnits.REV)
    side_end = rotation_strafe.position(RotationUnits.REV)
    print(gyro_start, ",", fwd_start, ",", side_start, ",", gyro_end, ",", fwd_end, ",", side_end)

    wait(1, SECONDS)

    gyro_start = inertial.rotation()
    smart_start = dt.rotation()
    print(gyro_start, smart_start)
    
    fwd_start = rotation_fwd.position(RotationUnits.REV)
    side_start = rotation_strafe.position(RotationUnits.REV)
    dt.turn_for(LEFT, 180.0, DEGREES, 33, PERCENT)
    dt.stop(COAST)
    wait(0.5, SECONDS)
    gyro_end = inertial.rotation()
    fwd_end = rotation_fwd.position(RotationUnits.REV)
    side_end = rotation_strafe.position(RotationUnits.REV)
    print(gyro_start, ",", fwd_start, ",", side_start, ",", gyro_end, ",", fwd_end, ",", side_end)

    # -0.06780071 , 0.2414444 , 0.8654722 , 722.9219 , 0.25 , -8.321305
    # 722.9198 , 0.25 , -8.321305 , -0.3253781 , 0.3342222 , 0.9289445

# drive_speed m/s
# circle_radius m
def sim_circle(drive_speed, circle_radius, slip_factor):            
    track_width = 9.5 * 25.4
    wheel_base = 10.0 * 25.4
    wheel_travel = (track_width**2 + wheel_base**2) * pi / track_width # m
    wheel_circum = 320.0
    wheel_revs_per_robot_rev = wheel_travel / wheel_circum
    external_gear_ratio = 1.0

    # drive_speed = 0.4 # m/s
    drive_distance = 2.0 * pi * circle_radius # m
    drive_time = drive_distance / drive_speed # m/s
    turn_speed = (360.0 / drive_time) * (pi / 180.0) # rad/s
    drive_command = external_gear_ratio * drive_speed / wheel_circum # motor turns / s
    turn_command = 1.0 * external_gear_ratio * wheel_revs_per_robot_rev * slip_factor / drive_time # motor turns / s

    left_rpm = (drive_command + turn_command) * 60.0 # BUGBUG
    right_rpm = (drive_command - turn_command) * 60.0 # BUGBUG

    print("Sim Circle:")
    print("- Drive Speed {:.1f} mm/s, Radius {:.1f} mm, Slip {:.1f}".format(drive_speed, circle_radius, slip_factor))
    print("- drive_distance {:.3f} m/s".format(drive_distance))
    print("- turn_speed {:.3f} rad/s".format(turn_speed))
    print("- drive_command {:.3f} rev/s".format(drive_command))
    print("- turn_command {:.3f} rev/s".format(turn_command))
    print("- left_rpm {:.3f} rpm".format(left_rpm))
    print("- right_rpm {:.3f} rpm".format(right_rpm))

    return left_rpm, right_rpm, drive_time

def auton4_circle_drive(drive_train: DriveProxy, tracker: Tracking):
    left_speed, right_speed, drive_time = sim_circle(250.0, 300.0, 0.44)
    log = []
    orientation = tracker.get_orientation()
    log.append([orientation.x,  orientation.y, orientation.heading])
    while drive_time > 0.0:
        # drive_train.spin(left_speed * 100.0 / 200.0, right_speed * 100.0 / 200.0) # convert from RPM to percent
        left_drive.spin(FORWARD, left_speed, RPM)
        right_drive.spin(FORWARD, right_speed, RPM)
        drive_time -= 0.01
        wait(10, MSEC)
        orientation = tracker.get_orientation()
        log.append([orientation.x,  orientation.y, orientation.heading])
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)    
    orientation = tracker.get_orientation()
    log.append([orientation.x,  orientation.y, orientation.heading])

    for entry in log:
        print(entry[0], ",", entry[1], ",", entry[2])
        wait(50, MSEC)

def test_concurrent(drive_train: DriveProxy, tracker: Tracking):
    print(drive_train.turn_for(RIGHT, 45, DEGREES, wait = False))
    while not drive_train.is_done(): wait(10, MSEC)
    print_tracker(tracker)
    print(drive_train.turn_for(LEFT, 45, DEGREES, wait = True))
    print_tracker(tracker)
    print(drive_train.drive_for(FORWARD, 50, MM, wait = False))
    try:
        drive_train.set_drive_velocity(10, PERCENT)
    except:
        print("Call to set_velocity failed")
    while not drive_train.is_done():wait(10, MSEC)
    print_tracker(tracker)
    print(drive_train.drive_for(REVERSE, 50, MM, wait = True))
    print_tracker(tracker)

def autonomous():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.clear_screen()
    brain.screen.print("auton")
    brain.screen.new_line()
    print("auton")

    drive_train = DriveProxy(left_drive, right_drive, inertial, wheel_travel_mm=DRIVETRAIN_WHEEL_SIZE)
    drive_train.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0, settle_error=0.5) # degrees
    drive_train.set_drive_constants(Kp=0.5, Ki=0.01, Kd=0.0, settle_error=5) # mm
    drive_train.set_heading_lock_constants(Kp=2.0, Ki=0.0, Kd=0.0) # degrees
    drive_train.set_turn_velocity(66, PERCENT)
    drive_train.set_drive_velocity(66, PERCENT)
    drive_train.set_drive_acceleration(10, PERCENT) # 5% per timestep

    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    tracker.enable()

    free = gc.mem_free() # type: ignore
    print(free)

    # calibration_tracking_wheels()
    # auton1_drive_straight(drive_train, tracker)
    # auton2_drive_to_points(drive_train, tracker)
    # auton3_drive_to_points_long(drive_train, tracker)
    # auton4_circle_drive(drive_train, tracker)
    # auton5_circle_follow(drive_train, tracker)
    test_concurrent(drive_train, tracker)

    print("auton done")

def user_control():
    print("user control")

    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    free = gc.mem_free() # type: ignore
    print(free)

    brain.screen.clear_screen()
    brain.screen.print("user control")
    brain.screen.new_line()

    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    tracker.enable()

    while True:
        print_tracker(tracker)
        wait(2, SECONDS)

comp = Competition(user_control, autonomous)
pre_autonomous()

