# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Nick                                                         #
# 	Created:      11/6/2025, 8:12:02 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
#   Demonstrates use of the V5PythonFramework library including                #
#   - InertialWrapper class for gyro scaling                                   #
#   - DriveProxy class drivetrain control                                      #
#   - SmartDriveWrapper class drivetrain control                               #
#   - Tracking class with motor odometry and tracking wheels                   #
#   - DriverControl class                                                      #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import gc
from math import pi, degrees, radians, sin, cos, atan2
from v5pythonlibrary import *

brain=Brain()
controller_1 = Controller(PRIMARY)

# Motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)
DRIVETRAIN_WHEEL_SIZE = 316.0
DRIVETRAIN_GEAR_RATIO = 1.0

all_motors = [l1, l2, r1, r2]
all_motor_names = ["l1", "l2", "r1", "r2"]

# Inertial Sensor
GYRO_SCALE_FOR_READOUT = 362.0/360.0
inertial = InertialWrapper(Ports.PORT5, GYRO_SCALE_FOR_READOUT)

# Rotation Sensors
rotation_fwd = Rotation(Ports.PORT6, False)
rotation_strafe = Rotation(Ports.PORT7, False)
all_sensors = [inertial, rotation_fwd, rotation_strafe]

# ------------------------------------------------------------ #
# Global State
# ------------------------------------------------------------ #
#
# These variables are used to control the cycling between pre-auton, autonomus and user control
# The flow is different depending on how the program is started:
#
# Case 1: Started from the controller by selecting the program and pressing "RUN"
# - in this case pre_autonomous() and user_control() start AT THE SAME TIME
# - this means any sensors particularly the inertial sensor will not finish calibrating before the user
#   gets control
# - ROBOT_INITIALIZED is used to block user control until pre_autonomous() is done
#
# Case 2: Started via VEX Field Control System or using TimedRun or Competiion selections on the controller
# - pre_autonomous() is called first
# - autonomous() or user_control() is called after 3 seconds in the case of TimedRun, or when the field
#   control system starts the match which can be an arbirarily long time
# - the corner case here is that if there is a UI selection in pre_autonomous(), we want that to finish
#   when autonomous() or user_control() starts
# - AUTON_STARTED and USER_STARTED are used to signal pre_autonomous() to wrap up any UI functionality
#
# ------------------------------------------------------------ #

ROBOT_INITIALIZED = False
ROBOT_INITIALIZATION_FAILED = False
AUTON_STARTED = False
USER_STARTED = False

# ------------------------------------------------------------ #
# Odometry / Tracking
# ------------------------------------------------------------ #

USING_TRACKING_WHEELS = False # sets the default tracker to use
USING_RESAMPLING = False

motor_tracker = None  # type: Tracking | None
odom_tracker = None  # type: Tracking | None
tracker = None # type: Tracking | None

def initialize_motor_tracker():

    ODOMETRY_FWD_SIZE = DRIVETRAIN_WHEEL_SIZE
    ODOMETRY_FWD_OFFSET = 0.0
    ODOMETRY_FWD_GEAR_RATIO = DRIVETRAIN_GEAR_RATIO
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 0.0

    tracker_configuration = Tracking.Configuration(
            fwd_is_odom=False,
            fwd_wheel_size=ODOMETRY_FWD_SIZE,
            fwd_gear_ratio=ODOMETRY_FWD_GEAR_RATIO,
            fwd_offset=ODOMETRY_FWD_OFFSET,
            side_wheel_size=ODOMETRY_STRAFE_SIZE,
            side_gear_ratio=ODOMETRY_STRAFE_GEAR_RATIO,
            side_offset=ODOMETRY_STRAFE_OFFSET
        )

    tracker_devices = [left_drive, right_drive, inertial]

    tracker = Tracking(tracker_devices, configuration=tracker_configuration, name="motor")
    tracker.enable_resampling(USING_RESAMPLING)
    # give tracker some time to get going
    wait(0.03, SECONDS)

    return tracker

def initialize_odom_tracker():

    # need to recalibrate these
    ODOMETRY_FWD_SIZE = 218.344 #219.70 from Remy
    ODOMETRY_FWD_OFFSET = 0.0316 * 25.4
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 157.38
    ODOMETRY_STRAFE_OFFSET = 4.526 * 25.4
    ODOMETRY_STRAFE_GEAR_RATIO = 1.0

    tracker_configuration = Tracking.Configuration(
            fwd_is_odom=True,
            fwd_wheel_size=ODOMETRY_FWD_SIZE,
            fwd_gear_ratio=ODOMETRY_FWD_GEAR_RATIO,
            fwd_offset=ODOMETRY_FWD_OFFSET,
            side_wheel_size=ODOMETRY_STRAFE_SIZE,
            side_gear_ratio=ODOMETRY_STRAFE_GEAR_RATIO,
            side_offset=ODOMETRY_STRAFE_OFFSET
        )

    tracker_devices = [rotation_fwd, rotation_strafe, inertial]
    rotation_fwd.set_position(0, RotationUnits.REV)
    rotation_strafe.set_position(0, RotationUnits.REV)

    tracker = Tracking(tracker_devices, configuration=tracker_configuration, name="odom")
    # Seems to work better with no resampling
    tracker.enable_resampling(USING_RESAMPLING)
    # give tracker some time to get going
    wait(0.03, SECONDS)

    return tracker

def change_tracker(new_tracker: Tracking, old_tracker: Tracking):
    global tracker
    if new_tracker is None or old_tracker is None:
        raise Exception("tracker not initialized")
    
    # note that this time is dependent on how long it takes for the robot to stop moving and sensors
    # particularly the inertial sensor to stabilize. when using COAST for stopping this can take seconds
    # but still seems to take surprisingly long with BRAKE
    wait(250, MSEC)

    print_tracker(old_tracker, 0, 0)

    location = old_tracker.get_orientation()
    old_tracker.enable(False)
    wait(20, MSEC)
    new_tracker.set_orientation(location, ignore_heading=True)
    wait(20, MSEC)
    new_tracker.enable(True)
    wait(20, MSEC)
    print_tracker(new_tracker, 0, 0)

    tracker = new_tracker
    return tracker

def print_tracker(tracker: Tracking, x = 0.0, y = 0.0, reverse=False, verbose = False):
    orientation = tracker.get_orientation()
    print("{} X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(tracker.name, orientation.x, orientation.y, orientation.heading))
    if verbose:
        origin_distance, origin_heading = tracker.trajectory_to_point(x, y, reverse=reverse)
        back_x, back_y = tracker.point_on_robot(-160.0, -7.5 * 25.4)
        print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))
        print(" - Back X: {:.1f} mm, Back Y: {:.1f} mm".format(back_x, back_y))
        print(" - Perf Avg Rate: {:.1f}ms, Avg Time: {:.1f} us".format(tracker._avg_rate, tracker._avg_time))

# ------------------------------------------------------------ #
# Pre-Autonomous Function
# ------------------------------------------------------------ #

def pre_autonomous():
    global ROBOT_INITIALIZED, ROBOT_INITIALIZATION_FAILED
    global tracker, motor_tracker, odom_tracker

    wait(0.1, SECONDS)

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
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

    motor_tracker = initialize_motor_tracker()
    odom_tracker = initialize_odom_tracker()
    tracker = odom_tracker if USING_TRACKING_WHEELS else motor_tracker

    brain.screen.print("pre auton -- done")
    brain.screen.new_line()
    print("pre_auton done")

    ROBOT_INITIALIZED = True

    # dummy wait to allow auton or user control to start
    # UI would run here is present
    if not AUTON_STARTED and not USER_STARTED:
        ui = PreAutonUI(brain)
        ui.start()
        while not AUTON_STARTED and not USER_STARTED:
            wait(100, MSEC)
        ui.stop()
        alliance, sequence = ui.get_current_selection() # gives our alliance color and auton sequence

    # optionally run the motor monitor
    # note right now this takes over the whole screen
    # TODO: cooperate with some other info like X / Y position etc.
    if False:
        motor_monitor = MotorMonitor(brain, all_motors, all_motor_names)
        motor_monitor.start()

# ------------------------------------------------------------ #
# Auton Routines for SmartDriveWrapper DriveTrain
#
# SmartDriveWrapper mimics the SmartDrive API but adds drive
# straight functionality as well as drive to point. SmartDriveWrapper
# is built on top of the lower level DriveProxy drivetrain
#
# For best results use the InertialWrapper class with a properly
# scaled gyro
# ------------------------------------------------------------ #

def drivetrain_max_speeds(motor_speed_rpm, wheel_size_mm, gear_ratio):
    '''
    ### Docstring for drivetrain_max_speeds
    
    :param motor_speed: in RPM (e.g. 600)
    :param wheel_size: circumference in MM (e.g. 260 for 3.25" wheels)
    :param gear_ratio: input gear / output gear (e.g. 24/60)

    :returns linear_speed, turn_speed: Tuple(linear_speed in mm/s, turn_speed in rev/s)
    '''
    linear_speed = motor_speed_rpm * wheel_size_mm * gear_ratio / 60.0 # will be in MM/S 
    turn_speed = 1.0 # hack this for now

    return linear_speed, turn_speed

def smart_drive_to_points(drivetrain: SmartDriveWrapper, tracker: Tracking):
    print("auton4_drive_to_points_long")

    drive_speed = 66 # PERCENT
    turn_speed = 66 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(200, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)

    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_stopping(BrakeType.BRAKE)

    print_tracker(tracker)
    
    if False:
        # small test rectangle
        # field tiles are 601mm to 602mm across in reality
        # wheel size etc. are all calibrated assuming 600mm tiles
        # X is NORTH-SOUTH, Y is EAST-WEST
        x_near = 0.0
        x_far = 2.0 * 600.0

        y_left = 0.0
        y_right = 1.0 * 600.0

        points = [
            [x_near, y_left], # start point
            [x_far, y_left],
            [x_far, y_right],
            [x_near, y_right],
            [x_near, y_left]
        ]
    else:
        # longer meander around the field
        # field tiles are 601mm to 602mm across in reality
        # wheel size etc. are all calibrated assuming 600mm tiles
        # X is NORTH-SOUTH, Y is EAST-WEST
        x_near = 1.5 * 600.0
        x_far = 4.5 * 600.0

        y_far_left = 0.5 * 600.0
        y_mid_left = 2.0
        y_mid_right = 4.0 * 600.0
        y_far_right = 5.5 * 600.0

        points = [
            [x_near, y_mid_left], # start point
            [x_far, y_mid_left],
            [x_far, y_mid_right],
            [x_near, y_mid_right],
            [x_near, y_far_left],
            [x_far, y_far_left],
            [x_far, y_far_right],
            [x_near, y_far_right],
            [x_near, y_mid_left]
        ]

    start_point = points.pop(0) # remove first point as that is the starting point
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    for i in range(4):
        for point in points:
            print("")
            print("----- Start Drive", i, point)
            x = point[0]
            y = point[1]
            print_tracker(tracker, x, y)
            print(inertial.rotation())
            distance, heading = tracker.trajectory_to_point(x, y)

            drive_timeout = 1.0 + distance / linear_speed_mm_sec # convert to MM/s and pad with 1 sec
            turn_timeout = 1.0 # HACK

            # Point in roughly the right direction
            drivetrain.set_timeout(turn_timeout, SECONDS)
            drivetrain.set_turn_threshold(1.0) # DEGREES - Can relax constrains before longer drives
            drivetrain.turn_to_heading(heading)
            print_tracker(tracker, x, y)

            # Drive straight
            distance, heading = tracker.trajectory_to_point(x, y)
            drivetrain.set_timeout(drive_timeout, SECONDS)
            drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
            wait(0.1, SECONDS)
            print_tracker(tracker, x, y)

    # Final Turn
    print("Start Turn")

    drivetrain.set_timeout(1.0 + turn_timeout, SECONDS)
    drivetrain.set_turn_threshold(0.25) # DEGREES - More accuracy here
    drivetrain.turn_to_heading(0.0)
    
    wait(0.1, SECONDS)
    print_tracker(tracker, start_point[0], start_point[1])

def position_callback():
    return tracker.x, tracker.y, 0.0 # type: ignore

def smart_drive_to_points_test(drivetrain: SmartDriveWrapper, tracker: Tracking):
    print("smart_drive_to_points_test")

    drive_speed = 66 # PERCENT
    turn_speed = 66 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(200, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)

    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_stopping(BrakeType.BRAKE)
    drivetrain.set_turn_threshold(1.0) # DEGREES - Can relax constrains before longer drives
    drivetrain.set_drive_threshold(10) # MM

    print_tracker(tracker)
    
    # small test rectangle
    # field tiles are 601mm to 602mm across in reality
    # wheel size etc. are all calibrated assuming 600mm tiles
    # X is NORTH-SOUTH, Y is EAST-WEST
    x_near = 0.0
    x_far = 1.0 * 600.0

    y_left = 0.0
    y_right = 1.0 * 600.0

    y_mid = (y_left + y_right) / 2.0

    points = [
        [x_near, y_left, FORWARD], # start point
        [x_far, y_left, FORWARD],
        [x_near, y_mid, REVERSE],
        [x_far, y_mid, FORWARD],
        [x_near, y_right, REVERSE],
        [x_far, y_right, FORWARD],
        [x_near, y_left, REVERSE]
    ]

    start_point = points.pop(0) # remove first point as that is the starting point
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    for i in range(1):
        for point in points:
            print("")
            x = point[0]
            y = point[1]
            dir = point[2]
            print("----- Start Drive", i, x, y)            
            print_tracker(tracker, x, y)
            distance, heading = tracker.trajectory_to_point(x, y)
            drive_timeout = 1.0 + distance / linear_speed_mm_sec # convert to MM/s and pad with 1 sec
            drivetrain.set_timeout(drive_timeout, SECONDS)
            drivetrain.drive_to_point(x, y, dir, position_callback)
            wait(0.1, SECONDS)
            print_tracker(tracker, x, y)

            wait(2, SECONDS)

    # Final Turn
    if True:
        print("Start Turn")

        turn_timeout = 1.0
        drivetrain.set_timeout(1.0 + turn_timeout * turn_speed / 100.0, SECONDS)
        drivetrain.set_turn_threshold(0.25) # DEGREES - More accuracy here
        drivetrain.turn_to_heading(0.0)
        
        wait(0.1, SECONDS)

    print_tracker(tracker, start_point[0], start_point[1])


def calibration_tracking_wheels(dt: SmartDriveWrapper):

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

def smart_turn_tests(drivetrain: SmartDriveWrapper, tracker: Tracking):
    print("smart_turn_tests")
    print_tracker(tracker)
    
    log_motors = [left_drive, right_drive]
    # log_motors = all_motors
    log = Logger(brain, log_motors + all_sensors, ["lmg", "rmg", "gyro", "fwd", "side"], data_headers=["x", "y"],
                 data_fields_callback=OnLoggerDataUpdate, time_sec=20, auto_dump=True, file_name="smart_turn_tests")
    # log.start()

    turn_angles = [22.5, -22.5, 45.0, -45.0, 90.0, -90.0, 180.0, -180.0, 360.0, -360.0]

    drivetrain.set_turn_velocity(100, PERCENT)
    # drivetrain.set_turn_constants(Kp=0.5, Ki=0.04, Kd=10.0)
    drivetrain.set_turn_constants(Kp=1.25/2.0, Ki=0.003, Kd=0.095)
    # drivetrain.set_turn_constant(0.5)

    for angle in turn_angles:
        #print("")
        #print("----- Start Turn", angle)
        initial_heading = tracker.get_orientation().heading
        target_heading = initial_heading + angle
        #print(" Initial Heading: {:.2f} deg, Target Heading: {:.2f} deg".format(initial_heading, target_heading))

        drivetrain.set_timeout(1.0 + abs(angle / 360.0), SECONDS)
        drivetrain.turn_for(RIGHT if angle > 0 else LEFT, abs(angle), DEGREES, mode=SmartDriveWrapper.TurnMode.PERCENT)
        drivetrain.stop(BrakeType.BRAKE)
        wait(0.5, SECONDS)

        final_heading = tracker.get_orientation().heading
        heading_error = final_heading - target_heading
        #print(" Final Heading: {:.2f} deg, Heading Error: {:.2f} deg".format(final_heading, heading_error))

def smart_drive_tests(tracker: Tracking):
    drivetrain = SmartDriveWrapper(left_drive, right_drive, inertial, DRIVETRAIN_WHEEL_SIZE, 320, 320, MM, DRIVETRAIN_GEAR_RATIO)

    drivetrain.set_drive_constants(Kp=0.5, Ki=0.01, Kd=0.0)
    drivetrain.set_drive_velocity(75, PERCENT)
    drivetrain.set_drive_acceleration(10, PERCENT)
    drivetrain.set_drive_threshold(10) # MM
    drivetrain.set_turn_velocity(75, PERCENT)
    drivetrain.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0)
    # drivetrain.set_turn_constants(Kp=1.25/2.0, Ki=0.003, Kd=0.095) # when using percent speed
    drivetrain.set_turn_threshold(0.5) # DEGREES
    drivetrain.set_heading_lock_constants(Kp=1.25, Ki=0.0, Kd=0.0)

    # smart_drive_to_points(drivetrain, tracker)
    # smart_turn_tests(drivetrain, tracker)
    smart_drive_to_points_test(drivetrain, tracker)
    
# ------------------------------------------------------------ #
# Auton Routines for DriveProxy DriveTrain
#
# These routines use the lower level DriveProxy drivetrain directly
# DriveProxy implements the actual drive control algorithms
# ------------------------------------------------------------ #

def OnLoggerDataUpdate():
    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    return tracker.x, tracker.y

def tracker_switch_test(drive_train: DriveProxy):
    global tracker
    if motor_tracker is None or odom_tracker is None or tracker is None:
        raise RuntimeError("Trackers not initialized")
    
    log_motors = [left_drive, right_drive]
    # log_motors = all_motors
    log = Logger(brain, log_motors + all_sensors, ["lmg", "rmg", "gyro", "fwd", "side"], data_headers=["x", "y"], data_fields_callback=OnLoggerDataUpdate, time_sec=30, auto_dump=True, file_name="tracker_switch_test")
    # log.fileio_test()
    log.start()

    tracker.enable(False)
    wait(20, MSEC)

    tracker = odom_tracker
    tracker.set_orientation(Tracking.Orientation(300.0, 300.0, 0.0))
    tracker.enable(True)

    wait (20, MSEC)

    points = [
        (1500.0, 300.0),
        (300.0, -300.0),
        (1500.0, -300.0),
        (300.0, 300.0)
    ]

    switch = False
    i = 0
    for point in points:
        x = point[0]
        y = point[1]
        print("----- To Point -----")
        print_tracker(tracker, x, y)

        distance, heading = tracker.trajectory_to_point(x, y)
        drive_train.turn_to_heading(heading)
        distance, heading = tracker.trajectory_to_point(x, y)
        drive_train.drive_for(FORWARD, distance, MM, heading)
        drive_train.stop(BRAKE)
        wait (200, MSEC)
        print_tracker(tracker, x, y, verbose=True)

        if (switch):
            if i % 2 == 1:
                print("----- Switch to Motor Tracker -----")
                motor_tracker.set_orientation(tracker.get_orientation(), ignore_heading=True)
                odom_tracker.enable(False)
                motor_tracker.enable(True)
                tracker = motor_tracker
            else:
                print("----- Switch to Odom Tracker -----")
                odom_tracker.set_orientation(tracker.get_orientation(), ignore_heading=True)
                motor_tracker.enable(False)
                odom_tracker.enable(True)
                tracker = odom_tracker

        wait(20, MSEC)
        print_tracker(tracker, x, y)
        i += 1

    drive_train.turn_to_heading(0)
    drive_train.stop(BRAKE)
    wait (200, MSEC)
    print_tracker(tracker, 300.0, 300.0, verbose=True)
    drive_train.stop(COAST)

    motor_tracker.stop_tracker()
    odom_tracker.stop_tracker()

    wait(100, MSEC)

    log.stop(True)

def dp_drive_to_points_long(drive_train: DriveProxy, tracker:Tracking):
    print("dp_drive_to_points_long")
    print_tracker(tracker)

    x_near = 0.0
    x_far = 2.0 * 600.0

    y_left = 0.0
    y_right = 1.0 * 600.0

    points = [
        [x_near, y_left], # start point
        [x_far, y_left],
        [x_far, y_right],
        [x_near, y_right],
        [x_near, y_left]
    ]
    '''
    # field tiles are 600mm across
    x_near = 1.5 * 600.0
    x_far = 4.5 * 600.0

    y_far_left = 0.5 * 600.0
    y_mid_left = 2.0
    y_mid_right = 4.0 * 600.0
    y_far_right = 5.5 * 600.0

    points = [
        [x_near, y_mid_left], # start point
        [x_far, y_mid_left],
        [x_far, y_mid_right],
        [x_near, y_mid_right],
        [x_near, y_far_left],
        [x_far, y_far_left],
        [x_far, y_far_right],
        [x_near, y_far_right],
        [x_near, y_mid_left]
    ]
    '''

    start_point = points.pop(0) # remove first point as that is the starting point
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    for i in range(4):
        for point in points:
            print("")
            print("----- Start Drive", i, point)
            x = point[0]
            y = point[1]
            print_tracker(tracker, x, y,reverse=True)
            print("inertial rotation:",  inertial.rotation())

            distance, heading = tracker.trajectory_to_point(x, y,reverse=True)
            print(" START TURN")
            drive_train.turn_to_heading(heading, settle_error=1.0, timeout=1.0)
            print_tracker(tracker, x, y,reverse=True)

            distance, heading = tracker.trajectory_to_point(x, y,reverse=True)
            timeout = 1.0 + abs(distance / (drive_train.linear_speed() * 1000.0)) # convert to MM/s and pad with 1 sec
            print(" START DRIVE")
            drive_train.drive_for(FORWARD, distance, MM, heading, timeout=timeout)
            wait(0.1, SECONDS)
            print_tracker(tracker, x, y,reverse=True)

    print("Start Turn")
    drive_train.turn_to_heading(0.0, settle_error=0.25, timeout=2.0)
    wait(0.1, SECONDS)
    print_tracker(tracker, start_point[0], start_point[1])

def OnTimeout():
    pass

def drive_proxy_tests(tracker: Tracking):
    drive_train = DriveProxy(left_drive, right_drive, inertial, wheel_travel_mm=DRIVETRAIN_WHEEL_SIZE)
    drive_train.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0, settle_error=0.5) # degrees
    drive_train.set_drive_constants(Kp=0.5, Ki=0.01, Kd=0.0, settle_error=5) # mm
    drive_train.set_heading_lock_constants(Kp=2.0, Ki=0.0, Kd=0.0) # degrees
    drive_train.set_turn_velocity(66, PERCENT)
    drive_train.set_drive_velocity(66, PERCENT)
    drive_train.set_drive_acceleration(10, PERCENT) # 5% per timestep

    # tracker_switch_test(drive_train)
    # calibration_tracking_wheels()
    dp_drive_to_points_long(drive_train, tracker)

# ------------------------------------------------------------ #
# Main Autonomous Function
# ------------------------------------------------------------ #

def autonomous():
    global AUTON_STARTED
    if (AUTON_STARTED):
        raise RuntimeError("autonomous called multiple times")
    AUTON_STARTED = True

    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.clear_screen()
    brain.screen.print("auton")
    brain.screen.new_line()
    print("auton")

    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    tracker.enable()

    # drive_proxy_tests(tracker)
    smart_drive_tests(tracker)

    free = gc.mem_free() # type: ignore
    print(free)

    print("auton done")

# ------------------------------------------------------------ #
# User Control
#
# All functionality is in the DriverControl class. Currenlty supported
# is left stick forward/reverse and right stick slow turn. Left stick can
# optionally be used for fast turn with a deadband to select when the left
# stick takes over.
#
# Accerleration limiting is built in to smooth out starts and stops and can
# be configured using the set_speed_limits() method.
#
# A rudimentary detwitch function for turns is provided to allow more accurate turns
# when turning in place (turns get faster as the more drive speed is applied).
#
# Optional heading hold and switching between VOLT and PERCENT control.
# ------------------------------------------------------------ #

def user_control():
    global USER_STARTED
    if (USER_STARTED):
        raise RuntimeError("user_control called multiple times")
    USER_STARTED = True

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

    drive_control = DriverControl(left_drive, right_drive, inertial)
    drive_control.set_mode(follow_heading_Kp=1.5)
    drive_control.set_mode(enable_drive_straight=True, enable_heading_lock=False, follow_heading=0.0)

    loop_count = 200
    while True:
        drive_control.user_drivetrain(controller_1.axis3.position(), controller_1.axis1.position())
        loop_count -= 1
        if (loop_count <= 0):
            print_tracker(tracker)
            loop_count = 200
        wait(10, MSEC)

comp = Competition(user_control, autonomous)
pre_autonomous()

