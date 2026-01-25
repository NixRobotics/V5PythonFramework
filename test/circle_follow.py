from vex import *
from math import pi
from inertialwrapper import InertialWrapper
from driveproxy import DriveProxy
from tracker import Tracking
from smartdrvwrapper import SmartDriveWrapper

brain = Brain()

RADIUS = 0.3       # meters
DRIVE_TIME = 10.0  # seconds
_start_time = None

def follow_circle(x, y, heading_deg):
    global _start_time
    if _start_time is None:
        _start_time = brain.timer.time() / 1000.0

    # convert from MM to M
    x = x / 1000.0
    y = y / 1000.0

    t = (brain.timer.time() / 1000.0) - _start_time
    omega = 2 * math.pi / DRIVE_TIME

    # Reference point on circle (X=NORTH, Y=EAST)
    x_ref = RADIUS * math.sin(omega * t)
    y_ref = RADIUS - RADIUS * math.cos(omega * t)

    # Tangent vector
    dx_ref = RADIUS * omega * math.cos(omega * t)
    dy_ref = RADIUS * omega * math.sin(omega * t)

    # Reference heading (clockwise from North)
    heading_ref = math.degrees(math.atan2(dy_ref, dx_ref))

    # Heading error (normalize to [-pi, pi])
    # heading_deg = InertialWrapper.to_angle(heading_deg)
    heading_err = InertialWrapper.to_angle(heading_ref - heading_deg)
    # print(heading_deg, heading_ref, heading_err)
    heading_err = math.radians(heading_err)

    #heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

    # Cross-track error
    cross_err_x = x_ref - x
    cross_err_y = y_ref - y
    cross_err = math.sqrt(cross_err_x*cross_err_x + cross_err_y*cross_err_y)

    # Signed using tangent vector
    sign = math.copysign(1, cross_err_x * (-dy_ref) + cross_err_y * dx_ref)
    cross_err *= sign

    # Control law
    base_speed = 25.0  # percent
    k_heading = 50.0
    k_cross = 20.0

    correction = k_heading * heading_err + k_cross * cross_err
    # correction = k_heading * heading_err
    left_speed = base_speed + correction
    right_speed = base_speed - correction

    # Clamp
    left_speed = max(min(left_speed, 100), 0)
    right_speed = max(min(right_speed, 100), 0)

    done = t >= DRIVE_TIME

    # print(x, y, heading_deg, left_speed, right_speed)

    return left_speed, right_speed, done

def auton5_circle_follow(drive_train: DriveProxy, tracker: Tracking):
    log = [(0.0,0.0,0.0)] * 10000
    orientation = tracker.get_orientation()
    log[0] = (orientation.x, orientation.y, orientation.heading)
    done = False
    i = 0
    while not done:
        orientation = tracker.get_orientation()
        if i < 10000:
            log[i] = (orientation.x,  orientation.y, orientation.heading)
            i += 1
        left_speed, right_speed, done = follow_circle(orientation.x,  orientation.y, orientation.heading)
        # drive_train.spin(left_speed, right_speed)
        
        free = gc.mem_free() # type: ignore
        print(free)

        wait(10, MSEC)
    drive_train.stop(COAST)

    for entry in log:
        print(entry[0], ",", entry[1])
        wait(50, MSEC)
