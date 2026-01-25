from vex import *
from v5pythonlibrary import *
from math import pi


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
    print(drive_train.turn_for(RIGHT, 90, DEGREES, wait = False))
    while not drive_train.is_done(): wait(10, MSEC)
    print_tracker(tracker)
    print(drive_train.turn_for(LEFT, 90, DEGREES, wait = True))
    print_tracker(tracker)
    print(drive_train.drive_for(FORWARD, 100, MM, wait = False))
    try:
        drive_train.set_drive_velocity(10, PERCENT)
    except:
        print("Call to set_velocity failed")
    while not drive_train.is_done():wait(10, MSEC)
    print_tracker(tracker)
    print(drive_train.drive_for(REVERSE, 100, MM, wait = True))
    print_tracker(tracker)

def OnOrientationUpdate():
    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    current_orientation = tracker.get_orientation()
    return current_orientation.x, current_orientation.y, current_orientation.heading

# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def auton2_drive_to_points(drive_train: DriveProxy, tracker: Tracking):
    print_tracker(tracker)

    drive_train.set_drive_velocity(50, PERCENT)
    drive_train.set_turn_velocity(50, PERCENT)
    drive_train.set_heading_lock_constants(Kp=1.0, Ki=0.0, Kd=0.0) # degrees
    for i in range(4):
        drive_train.drive_to_point(24.0 * 25.4, 0.0, FORWARD, OnOrientationUpdate, wait=True)
        drive_train.drive_to_point(24.0 * 25.4, 24.0 * 25.4, FORWARD, OnOrientationUpdate, wait=True)
        drive_train.drive_to_point(0.0, 24.0 * 25.4, FORWARD, OnOrientationUpdate, wait=True)
        drive_train.drive_to_point(0.0, 0.0, FORWARD, OnOrientationUpdate, wait=True)

    print_tracker(tracker)

    '''
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
    '''


# DEMO1: Once robot has been tuned for individual commands this will turn the robot and drive forward and backwards
def auton1_drive_straight(drive_train: DriveProxy, tracker: Tracking):
    drive_train.turn_to_heading(90.0, timeout=2.0)
    drive_train.turn_to_heading(0.0, timeout=2.0)

    distance = 36.0 * 25.4
    timeout = 1.0 + distance / (drive_train.linear_speed() * 1000.0) # convert to MM/s and pad with 1 sec
    drive_train.drive_for(FORWARD, distance, MM, 0.0,timeout=timeout)
    drive_train.drive_for(REVERSE, distance, MM, 0.0, timeout=timeout)
    
    drive_train.turn_to_heading(0.0, timeout=2.0)
    