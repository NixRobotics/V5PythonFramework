# V5PythonFramework

WIP

# Instructions:

Place following files on SDCard (local edits in VSCode will not be reflected):
- driveproxy.py
- inertialwrapper.py
- pid.py
- tracker.py
- smrtdriverwrapper.py

Optionally also copy the following files to the SDCard for additional functionality:
- autonhelpers.py
- drivercontrol.py
- logger.py
- motormonitor.py

Note that for the demo main.py included with this library, all the above files need to be present
on the SDCard.

main.py will be downloaded as usual.

# Including In Your Own Program

The only file you need to include in your project is the stubs/v5pythonlibrary.py. This basically provides the
equivalent of a header file to keep the python parser happy.

Starting a new project is similar to the basic VEX flow in Visiual Studio Code (VSCode). In the GUI navigate to the
"New Project" menu under the VEX extension, select VRC V5 and then Python. Best is always to create a new Competition Template.
This will create the required files such as the vex_project_settings.json and main.py. Up to now this is just the same
as creating any new V5 project in VSCode.

To include the library, you only need to do two things:
1. Copy the stubs/v5pythonlibrary.py file to the same directory as your main.py (do not copy the src/v5pythonlibrary.py file!)
2. Under the first line in main.py that should read "from vex import *" add the line "from v5pythonlibrary import *"

The first thing to try then is to use InertialWrapper instead of Inertial for the Inertial Sensor and SmartDriveWrapper
instead of SmartDrive for the drive train, e.g.:

from vex import *
from v5pythonlibrary import *

l1 = Motor(Ports.PORT1)
l2 = Motor(Ports.PORT2)
r1 = Motor(Ports.PORT3)
r2 = Motor(Ports.PORT4)
left_drive = MotorGroup(l1, l2)
right_drive = MotorGroup(r1, r2)

gyro_scale = 1.01 # will vary with each sensor
inertial = InertialWrapper(Ports.PORT5, gyro_scale)

dt = SmartDriveWrapper(left_drive, right_drive, inertial, etc.)
...

dt at this point should behave more or less the same as if SmartDrive() had been used directly, at least for the
turn_for() and drive_for() calls. The caveat is that the PID tuning parameters will be different and may need to
be adjusted. If you only want to adjust the proportional gain dt.set_drive_constant() and dt.set_turn_constant()
can be used.

# Caveats

Only provides basic turn_for and drive_for functionality. drive_for with heading specified will enable heading
lock, however only works reliably if robot is already pointing in more or less the right direcion. Erratic motion
may result if robot is too far off heading.

Does not do perform any drive to target pose algorithms such as boomerang, look-ahead or pure-pursuit. Although it
does have drive_to_point() functionality with fast exit once the perpendicular line to the target point as been crossed.

# Tracking Basics

For a good overview of the theory see this paper: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

This example does not implement this scheme directly, particularly the local to global rotation.

What is not discussed adequately here is the type of motion that can be tracked. Using two tracking wheels (i.e.
for forward and strafe/sideways) assumes that the robot turns are centered around its pivot point (point around which
the robot turns when turning in place). This is not the geometric center of the robot and not necessarily the geometric
center of the drivetrain wheels. When using traction wheels it will tend to be towards the center of these.

However, the pivot point will depend on the center of gravity (CoG) and the rotation speed. E.g. a robot with an
off center CoG will likely rotate around a point towards the geometric center of the drivetrain wheels when turning
slowly. As turn speed increases the pivot point will move towards the CoG. As the pivot point moves, the tracking
wheel offsets would need to be updated to correctly track the motion, but of course we don't have the ability to
detect this.

The other implication of this is that we can not track motion that intentionally changes the pivot point, e.g. by
manually rotating the robot around its back left corner. This is the kind of motion that would occur when the robot
is blocked against a field element or the field perimeter. These kind of interations change the center of rotation
enough that the assumptions behind the tracking equations are incorect.

# Calibration

Calibrate gyro and tracking wheels separately (spin robot left and right by sevaral revolutions to determine 
gyro error and tracking wheel offsets from robot pivot point).

Tracking wheel offsets from pivot point can be calculated using simple circle formula, ie:
- Tracking wheels at pivot point will be completely stationary during revolution
- Any offset from pivot point will show up as motion - this is the circle circumfrence the tracking wheel is tracing out
- Simple then to deduce offset (radius) of wheel from pivot point (C = 2.pi.R)
- R is your tracking wheel offset
- C is number of revolution of tracking wheel * wheel travel
- Make sure you have calibrated tracking wheel travel
- Assumes robot more or less turns around a consistent point (very rarely the case). Take multiple readings and
average
- NOTE: Being off on the forward and side/strafe tracking wheel offsets does not invalidate the math. You are
just tracking a different point on the robot. This will typically show up as an angle error to the next
waypoint. Another way of dealing with offset is to set both offsets to zero and use a local to global rotation
matrix to transform robot location for any arbitraty offset when needed. With offsets of zero for both wheels
the tracking math will track the point where the forward and strafe wheels intersect.

Sample transformation based on current tracked robot pose (x, y, theta) using NumPy:

class SimRobot:

    @staticmethod
    def rotation_local_to_global(angle):
        R = np.array([
            [cos(angle), -sin(angle), 0.0],
            [sin(angle), cos(angle), 0.0],
            [0.0, 0.0, 1.0]
        ])
        return R
    
    @staticmethod
    def point_on_robot(pose, offset):
        angle = pose[2]
        pose = np.array([[pose[0]], [pose[1]], [pose[2]]]) # column vector notation
        R = SimRobot.rotation_local_to_global(angle)
        point = pose + R @ offset
        return point

VEX does not enable overload of rotation sensor sample rate in Python, so sample interval is limited to 20ms.
Robot travelling at 1m/s will cover 2cm (20mm) in 20ms. Use your own judgement if this is acceptable or not
for your needs and adjust robot speed accordingly
- Linear interpolation to align rotation sensors in ime is not implemented in this version
- Motor encoders update at 10ms

Wheel travel for drivetrain and tracking can be be determined by driving robot sufficently far and comparing
reported revolutions and distance. Be careful to take any hysteris into account, e.g. if robot stops short of
target, but can be pushed to target without feeling motors - you will have this amount of uncertainty in your
wheel travel. Make sure to perform multiple experiments both backwards and forwards and average.

# InertialWrapper Class

This provides an overload for the rotation(), heading() and angle() inertial sensor calls as well as some of the
setter functions.

Not all combinations of calls are tested.

Using this you can specify your gyro error directly when you create the inertial sensor object, rather than
having to keep track of it separately:

inertial = InertialWrapper(port_number, gyro_scale)

The gyro_scale in this case is the READOUT gyro error, meaning the scale you need to apply to get a call such as
rotation() to provide a value that matches the physical robot rotation. The Tracking and DriveProxy classes do
the right thing based on this class.

For example, if your robot turns 365.0deg when commanded to turn 360.0 it means that the gyro is reporting a
heading that is too small. In this case the gyro_scale would be 365.0 / 360.0.

InertialWrapper can used with VEX SmartDrive as well, e.g

inertrial = InertialWrapper(port_number, gyro_scale)
dt = SmartDrive(left, right, inertial, ...)
heading = dt.heading() # heading will be provided by the override in InertialWrapper

# Class Library Overviews

Tracking, SmartDriveWrapper and InertialWrapper should be the most commonly used classes from this library. The
rough hierarchy for each is shown below

InertialWrapper
-> Inertial

SmartDriveWrapper
-> SmartDrive
    -> DriveProxy
        -> PID

Tracking

Note that for SmartDrive, almost none of the base functionality is actually used. Most calls are routed directly to
DriveProxy, so SmartDrive really just acts as a small shim layer to allow for easy migration of programs

