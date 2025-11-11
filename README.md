# V5PythonFramework

WIP

# Instructions:

Place following files on SDCard (local edits in VSCode will not be reflected)
- driveproxy.py
- inertialwrapper.py
- pid.py
- tracker.py

main.py will be downloaded as usual.

# Caveats

Only provides basic turn_for and drive_for functionality. drive_for with heading specified will enable heading
lock, however only works reliably if robot is already pointing in more or less the right direcion. Erratic motion
will result if robot is too far off heading (drive speed limiting based on heading error is not implemented).

Does not do perform any boomerang, look-ahead or pure-pursuit drive algorithms.

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

InertialWrapper is not compatible with VEX SmartDrive. If you want to call the VEX SmartDrive functions, you
need to also instantiate a separate inertial sensor object, ie.

inertial_for_smartdrive = Inertial(port_number)

For SmartDrive, gyro_scale value needs to be inverted, ie:
- If a robot overturns given a certain command such as SmartDrive.turn_for() it means the inertial sensor is
reading a too small value, so you need to provide SmartDrive.turn_for() a larger value
- I refer to this as the TURN gyro scale to distiguish it from the READOUT scale (TURN scale = 1.0 / READOUT scale)
  
