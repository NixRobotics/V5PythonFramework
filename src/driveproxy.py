from vex import *
from inertialwrapper import InertialWrapper
from pid import PID

# TODO: MOVE THESE
DRIVETRAIN_MOTOR_SPEED_RPM = 200.0
DRIVETRAIN_WHEEL_SIZE = 320.0
DRIVETRAIN_EXT_GEAR_RATIO = 60.0 / 60.0

# Simple drivetrain proxy. This replaces the VEX provided DriveTrain or SmartDrive classes with one that uses the inertial
# sensor for turning and optionally for driving straight. It uses the SimplePID class
# There are 3 sets of PID parameters needed to make this work
# - Turning: see set_turn_*()
# - Driving: see set_drive_*()
# - Heading lock while driving: see set_heading_lock_*(). Note this is different from pure turns and runs at the same time as driving
# All functiionality is implemented in the turn_for() and drive_for() functions
class DriveProxy:

    MAX_VOLTAGE = 11.5
    MAX_PERCENT = 50.0 # HACK: for same K values tuned for voltage, need to slow percent motor control down
    USE_VOLTAGE = True

    class PIDParameters:
        def __init__(self):
            self.Kp = 1.0
            self.Ki = 0.0
            self.Kd = 0.0
            self.max_output = 1.0
            self.max_ramp = 1.0
            self.settle_error = 1.0

    def __init__(self, left_motors: MotorGroup, right_motors: MotorGroup, inertial: InertialWrapper,
                 motor_speed=DRIVETRAIN_MOTOR_SPEED_RPM,
                 wheel_travel_mm=DRIVETRAIN_WHEEL_SIZE,
                 ext_gear_ratio=DRIVETRAIN_EXT_GEAR_RATIO):
        self.turn_pid_constants = DriveProxy.PIDParameters()
        self.drive_pid_constants = DriveProxy.PIDParameters()
        self.heading_lock_pid_constants = DriveProxy.PIDParameters()
        self.default_timeout = 10.0 # seconds

        self.left_motors = left_motors # motor group
        self.right_motors = right_motors # motor group
        self.motor_speed = motor_speed # RPM
        self.wheel_travel_mm = wheel_travel_mm
        self.ext_gear_ratio = ext_gear_ratio

        self.inertial = inertial

        self.stop_mode = BrakeType.COAST

        self.drive_velocity = 100.0 # percent

        self._worker_thread = None
        self._command_running = False
        self._cancel_command = False

    def is_done(self):
        return not self._command_running
    
    def _concurrency_check(self):
        if self._command_running:
            raise RuntimeError("Conncurrency check: Command already running")
            # return False
        return True

    def set_drive_velocity(self, velocity, unit):
        self._concurrency_check()
        self.drive_velocity = velocity
        self.drive_pid_constants.max_output = velocity / 100.0
        return True

    def set_drive_acceleration(self, acceleration, unit):
        self._concurrency_check()
        self.drive_pid_constants.max_ramp = acceleration / 100.0
        return True

    # settle error will be in MM, we need to convert to degree revolutions for internal use
    def calc_drive_settle_error(self, settle_error):
        return 360.0 * settle_error / (self.wheel_travel_mm * self.ext_gear_ratio)

    def set_drive_constants(self, Kp, Ki, Kd, settle_error):
        self._concurrency_check()
        self.drive_pid_constants.Kp = Kp
        self.drive_pid_constants.Ki = Ki
        self.drive_pid_constants.Kd = Kd
        self.drive_pid_constants.settle_error = self.calc_drive_settle_error(settle_error)
        return True

    def set_turn_velocity(self, velocity, unit):
        self._concurrency_check()
        self.turn_pid_constants.max_output = velocity / 100.0
        self.heading_lock_pid_constants.max_output = velocity / 100.0
        return True

    def set_turn_acceleration(self, acceleration, unit):
        self._concurrency_check()
        self.turn_pid_constants.max_ramp = acceleration / 100.0
        return True

    def set_turn_constants(self, Kp, Ki, Kd, settle_error):
        self._concurrency_check()
        self.turn_pid_constants.Kp = Kp
        self.turn_pid_constants.Ki = Ki
        self.turn_pid_constants.Kd = Kd
        # degrees
        self.turn_pid_constants.settle_error = settle_error
        return True

    def set_heading_lock_constants(self, Kp, Ki, Kd, settle_error):
        self._concurrency_check()
        self.heading_lock_pid_constants.Kp = Kp
        self.heading_lock_pid_constants.Ki = Ki
        self.heading_lock_pid_constants.Kd = Kd
        self.heading_lock_pid_constants.settle_error = settle_error
        return True

    def set_timeout(self, time):
        self._concurrency_check()
        self.default_timeout = time
        return True

    def set_stopping(self, mode):
        self._concurrency_check()
        self.stop_mode = mode
        return True

    # Returns approx max linear speed of robot (m/s) - useful for timeout calculations
    # If using voltage for motors, top speed will be somewhat above stated RPM of cartridge
    # Will not take into account acceleration, deceleration and settle time - pad appropriately
    def linear_speed(self):
        # Motor speed in RPM and wheel size in MM
        return self.motor_speed * self.ext_gear_ratio * self.wheel_travel_mm * self.drive_velocity / (1000.0 * 60.0 * 100.0)
    
    # Approximate max rotational velocity of robot in deg/s
    # Using most common builds having mix of traction and omni wheels a 200RPM drive on 4" wheels should be
    # able to turn 360deg in 1 sec, so we scale this heuristic appropriately
    # TODO: FIXME
    def rotation_speed(self):
        reference_linear_speed = 1.067 # m/s - speed of 200RPM drive using 4" wheels and 1:1 gear ratio
        reference_rotation_speed = 360.0 # deg/s
        return reference_rotation_speed * self.linear_speed() / reference_linear_speed

    def turn_to_heading(self, heading, settle_error = None, timeout = None, wait = True):
        angle = self.inertial.calc_angle_to_heading(heading)
        return self.turn_for(RIGHT, angle, DEGREES, settle_error=settle_error, timeout=timeout, wait=wait)

    def _turn_for(self, direction, angle, unit, settle_error = None, timeout = None):
        if unit is not RotationUnits.DEG: raise NotImplementedError("Units must be MM")
        self._command_running = True
        timer = Timer()

        turn_pid = PID(self.turn_pid_constants.Kp, self.turn_pid_constants.Ki, self.turn_pid_constants.Kd)
        turn_pid.set_output_limit(self.turn_pid_constants.max_output) # limit output to defined power
        turn_pid.set_output_ramp_limit(self.turn_pid_constants.max_ramp)
        # allow for per call settle_threshold and timeout, useful if we need to vary accuracy particularly when chaining motions
        turn_pid.set_settle_threshold(self.turn_pid_constants.settle_error if settle_error is None else settle_error) # settle threshold in degrees
        turn_pid.set_timeout(self.default_timeout if timeout is None else timeout)
        start_rotation = self.inertial.rotation()
        target_rotation = start_rotation + (angle if direction == TurnType.RIGHT else -angle)
        while not turn_pid.is_done() and not self._cancel_command:
            current_rotation = self.inertial.rotation()
            pid_output = turn_pid.compute(target_rotation, current_rotation)

            self._spin(pid_output, -pid_output)

            wait(turn_pid.timestep, SECONDS)

        self._stop(self.stop_mode)
        print("Done Turn: ", turn_pid.get_is_settled(), turn_pid.get_is_timed_out())

        # for log_entry in turn_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)
        self._command_running = False
        return timer.time()
    
    def _turn_for_thread(self, args1, arg2, arg3, arg4, arg5):
        self._turn_for(args1, arg2, arg3, arg4, arg5)
    
    def turn_for(self, direction, angle, unit, settle_error = None, timeout = None, wait = True):
        self._concurrency_check()
        self._command_running = True
        if wait:
            return self._turn_for(direction, angle, unit, settle_error, timeout)
        else:
            self._worker_thread = Thread(self._turn_for_thread, (direction, angle, unit, settle_error, timeout))
            return 0

    def _drive_for(self, direction, distance, unit, heading = None, settle_error = None, timeout = None):
        if unit is not DistanceUnits.MM: raise NotImplementedError("Units must be DEG")
        self._command_running = True
        timer = Timer()

        drive_pid = PID(self.drive_pid_constants.Kp, self.drive_pid_constants.Ki, self.drive_pid_constants.Kd)
        drive_pid.set_output_limit(self.drive_pid_constants.max_output) # limit output to 50% power
        drive_pid.set_output_ramp_limit(self.drive_pid_constants.max_ramp)
        # see if we want to override settle and timeout
        if (settle_error is None): drive_pid.set_settle_threshold(self.drive_pid_constants.settle_error)
        else: drive_pid.set_settle_threshold(self.calc_drive_settle_error(settle_error))
        drive_pid.set_timeout(self.default_timeout if timeout is None else timeout)

        if (heading is not None):
            turn_pid = PID(self.heading_lock_pid_constants.Kp, self.heading_lock_pid_constants.Ki, self.heading_lock_pid_constants.Kd)
            turn_pid.set_output_limit(self.heading_lock_pid_constants.max_output) # limit output to 50% power
            turn_pid.set_settle_time(0.0)
            turn_pid.set_timeout(0.0)

            target_rotation = self.inertial.calc_rotation_at_heading(heading)

        left_start_pos = self.left_motors.position(RotationUnits.DEG)
        right_start_pos = self.right_motors.position(RotationUnits.DEG)

        target_distance_revs = 360.0 * distance / (self.wheel_travel_mm * self.ext_gear_ratio) # convert mm to wheel revolutions assuming 100mm diameter wheels
        target_position = target_distance_revs if direction == DirectionType.FORWARD else -target_distance_revs

        while not drive_pid.is_done() and not self._cancel_command:
            current_position = (
                (self.left_motors.position(RotationUnits.DEG) - left_start_pos) +
                (self.right_motors.position(RotationUnits.DEG) - right_start_pos)) / 2.0

            pid_output = drive_pid.compute(target_position, current_position)

            turn_pid_output = 0.0
            if (heading is not None):
                current_rotation = self.inertial.rotation()
                turn_pid_output = turn_pid.compute(target_rotation, current_rotation)

            self._spin(pid_output + turn_pid_output, pid_output - turn_pid_output)

            wait(drive_pid.timestep, SECONDS)

        self._stop(self.stop_mode)
        print("Done Drive: ", drive_pid.get_is_settled(), drive_pid.get_is_timed_out())

        # for log_entry in drive_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)
        self._command_running = False
        return timer.time()

    def _drive_for_thread(self, args1, arg2, arg3, arg4, arg5, arg6):
        self._drive_for(args1, arg2, arg3, arg4, arg5, arg6)
    
    def drive_for(self, direction, distance, unit, heading = None, settle_error = None, timeout = None, wait = True):
        self._concurrency_check()
        self._command_running = True
        if wait:
            return self._drive_for(direction, distance, unit, heading, settle_error, timeout)
        else:
            self._worker_thread = Thread(self._drive_for_thread, (direction, distance, unit, heading, settle_error, timeout))
            return 0

    def drive_to_point(self, x, y, tracker):
        pass

    def _spin(self, left_speed, right_speed):
        if (self.USE_VOLTAGE):
            left_voltage = self.limit(left_speed * DriveProxy.MAX_VOLTAGE, DriveProxy.MAX_VOLTAGE)
            right_voltage = self.limit(right_speed * DriveProxy.MAX_VOLTAGE, DriveProxy.MAX_VOLTAGE)

            self.left_motors.spin(FORWARD, left_voltage, VOLT) # type: ignore
            self.right_motors.spin(FORWARD, right_voltage, VOLT) # type: ignore
        else:
            left_percent = self.limit(left_speed * DriveProxy.MAX_PERCENT, DriveProxy.MAX_PERCENT)
            right_percent = self.limit(right_speed * DriveProxy.MAX_PERCENT, DriveProxy.MAX_PERCENT)

            self.left_motors.spin(FORWARD, left_percent, PERCENT)
            self.right_motors.spin(FORWARD, right_percent, PERCENT)

    def spin(self, left_speed, right_speed):
        self._concurrency_check()
        self._spin(left_speed, right_speed)
        return True

    def _stop(self, mode):
        # Note that setting mode to None will keep motors at their last commanded output
        if (mode is not None):
            self.left_motors.stop(mode)
            self.right_motors.stop(mode)

    def stop(self, mode):
        if (self._command_running): self._cancel_command = True
        wait(10, MSEC)
        self._stop(mode)
        self._cancel_command = False

    def limit(self, input, limit_value):
        if (input > limit_value): return limit_value
        elif (input < -limit_value): return -limit_value
        return input

# ----------------------------
