# V5PythonFramework

WIP

# Instruction:

Place following files on SDCard (local edits in VSCode will not be reflected)
- driveproxy.py
- inertialwrapper.py
- pid.py
- tracker.py

main.py will be downloaded as usual.

# Caveats

Only provides basic turn_for and drive_for functionality. drive_for with heading spedified will enable heading
lock, however only works reliably if robot is already pointed in more or less the same direcion. Erratic motion
will result if robot is too far off heading (drive speed limiting based on heading error is not implemented).

Doess not do perform amy lookahead or pure pursuit drive algorithms.

Calibrate gyro and tracking wheels separately (spin robot left and right by sevaral revolutions to determine 
gyro error and tracking wheel offsets from robot pivot point).

Tracking wheel offsets from pivot point can be calculated using simple circle formula, ie:
- Tracking wheels at pivot point will be completely stationary during revolution
- Any offset from pivot point will show up as motion - this is the circle circumfrence the tracking wheel is tracing out.
- Simple then to deduce offset (radius) of wheel from pivot point (C = 2.pi.R)
- R is your tracking wheel offset
- C is number of revolution of tracking wheel * wheel travel
- Make sure you have calibrated tracking wheel travel
