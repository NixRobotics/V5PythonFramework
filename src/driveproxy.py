from vex import *
from inertialwrapper import InertialWrapper
from pid import PID
from math import degrees, radians, sin, cos, atan2

# Default Values
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
    MAX_PERCENT = 100.0
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
        self.heading_lock_pid_constants.settle_error = 0.0
        self.default_timeout = 10.0 # seconds

        self.left_motors = left_motors # motor group
        self.right_motors = right_motors # motor group
        self.motor_speed = motor_speed # RPM
        self.wheel_travel_mm = wheel_travel_mm
        self.ext_gear_ratio = ext_gear_ratio

        self.inertial = inertial

        self.stop_mode = BrakeType.COAST

        self.drive_velocity = DriveProxy.MAX_PERCENT # percent

        self._this_timeout = None # timeout of current command in seconds (or None if no command running)
        self._was_timeout = False # Indicates if last command timed out 
        self._timeout_notifier = Event()
        self._worker_thread = None
        self._command_running = False
        self._cancel_command = False

    def is_done(self):
        return not self._command_running
    
    def is_timed_out(self):
        return self._was_timeout
    
    def _concurrency_check(self):
        if self._command_running:
            raise RuntimeError("Conncurrency check: Command already running")
            # return False
        return True

    def set_drive_velocity(self, velocity, unit):
        if (unit is not PercentUnits.PERCENT): raise ValueError("Units must be PERCENT")
        self._concurrency_check()
        self.drive_velocity = velocity
        self.drive_pid_constants.max_output = velocity / 100.0
        return True

    def set_drive_acceleration(self, acceleration, unit: VelocityPercentUnits):
        if (unit is not PercentUnits.PERCENT and unit is not VelocityUnits.PERCENT): raise ValueError("Units must be PERCENT")
        self._concurrency_check()
        self.drive_pid_constants.max_ramp = acceleration / 100.0
        return True

    # settle error will be in MM, we need to convert to degree revolutions for internal use
    def calc_drive_settle_error(self, settle_error):
        '''
        ### Converts settle error in MM to degrees of wheel rotation
        '''
        return 360.0 * settle_error / (self.wheel_travel_mm * self.ext_gear_ratio)

    def set_drive_constants(self, Kp=None, Ki=None, Kd=None, settle_error=None):
        '''
        ### Sets the drive PID constants

        Internally PID is based on motor rotations, but settle_error is provided in MM for convenience. It is converted internally.

        ### Arguments
            Kp (optional): Proportional constant
            Ki (optional): Integral constant
            Kd (optional): Derivative constant
            settle_error (optional): settle error in MM. Default internally is 1 degree of motor rotation

        ### Returns
            True if successful
        '''
        self._concurrency_check()
        if Kp is not None: self.drive_pid_constants.Kp = Kp
        if Ki is not None: self.drive_pid_constants.Ki = Ki
        if Kd is not None: self.drive_pid_constants.Kd = Kd
        if settle_error is not None: self.drive_pid_constants.settle_error = self.calc_drive_settle_error(settle_error)
        return True

    def set_turn_velocity(self, velocity, unit):
        if (unit is not PercentUnits.PERCENT): raise ValueError("Units must be PERCENT")
        self._concurrency_check()
        self.turn_pid_constants.max_output = velocity / 100.0
        self.heading_lock_pid_constants.max_output = velocity / 100.0
        return True

    def set_turn_acceleration(self, acceleration, unit):
        if (unit is not PercentUnits.PERCENT): raise ValueError("Units must be PERCENT")
        self._concurrency_check()
        self.turn_pid_constants.max_ramp = acceleration / 100.0
        return True

    def set_turn_constants(self, Kp=None, Ki=None, Kd=None, settle_error=None):
        '''
        ### Sets the turn PID constants

        settle_error is in degrees of robot turns. Default is +/- 1deg\\
        Uses the current timeout value
        '''
        self._concurrency_check()
        if Kp is not None: self.turn_pid_constants.Kp = Kp
        if Ki is not None: self.turn_pid_constants.Ki = Ki
        if Kd is not None: self.turn_pid_constants.Kd = Kd
        # degrees
        if settle_error is not None: self.turn_pid_constants.settle_error = settle_error
        return True

    def set_heading_lock_constants(self, Kp=None, Ki=None, Kd=None):
        '''
        ### Sets the heading lock/hold constants

        Typically only need Kp which would be set higher than for just pure turns. Does not take a settle error and does
        not timeout
        '''
        self._concurrency_check()
        if Kp is not None: self.heading_lock_pid_constants.Kp = Kp
        if Ki is not None: self.heading_lock_pid_constants.Ki = Ki
        if Kd is not None: self.heading_lock_pid_constants.Kd = Kd
        return True

    def set_timeout(self, time):
        '''
        ### Sets timeout in SECONDS for all motion commands
        
        ### Arguments
            time: timeout in SECONDS
        '''
        self._concurrency_check()
        self.default_timeout = time
        return True
    
    def get_timeout(self):
        '''
        ### Gets the timeout of the curent command, or default if no command is executing

        ### Arguments
            None

        ### Returns
            Timeout is SECONDS
        '''
        return self.default_timeout if self._this_timeout is None else self._this_timeout
    
    def set_timeout_callback(self, fn):
        self._timeout_notifier.set(fn)

    def set_stopping(self, mode):
        self._concurrency_check()
        self.stop_mode = mode
        return True

    def linear_speed(self):
        '''
        ### Returns approx max linear speed of robot (m/s)
        
        Useful for timeout calculations.\\
        If using voltage for motors, top speed will be somewhat above stated RPM of cartridge.\\
        Will not take into account acceleration, deceleration and settle time - pad appropriately.
        '''
        # Motor speed in RPM and wheel size in MM
        return self.motor_speed * self.ext_gear_ratio * self.wheel_travel_mm * self.drive_velocity / (1000.0 * 60.0 * 100.0)
    
    def rotation_speed(self):
        '''
        ### Approximate max rotational velocity of robot in deg/s
        
        Using most common builds having mix of traction and omni wheels a 200RPM drive on 4" wheels should be
        able to turn 360deg in 1 sec, so we scale this heuristic appropriately

        #### TODO: FIXME
        '''
        reference_linear_speed = 1.067 # m/s - speed of 200RPM drive using 4" wheels and 1:1 gear ratio
        reference_rotation_speed = 360.0 # deg/s
        return reference_rotation_speed * self.linear_speed() / reference_linear_speed
    
    def test_finish_line(self, target_x, target_y, target_angle, current_x, current_y):
        '''
        test_finish_line checks if the robot has crossed the line perpendicular to the path to the target point
        Provides a way to stop robot hunting for the target point when it gets close, avoiding large swings in angle
        
        :param target_x: Description
        :param target_y: Description
        :param target_angle: Description
        :param current_x: Description
        :param current_y: Description
        '''
        return( (target_y-current_y) * sin(radians(target_angle)) <= -(target_x-current_x) * cos(radians(target_angle)) )

    def turn_to_heading(self, heading, settle_error = None, timeout = None, wait = True):
        angle = self.inertial.calc_angle_to_heading(heading)
        return self.turn_for(RIGHT, angle, DEGREES, settle_error=settle_error, timeout=timeout, wait=wait)

    def turn_to_rotation(self, rotation, settle_error = None, timeout = None, wait = True):
        angle = self.inertial.calc_angle_to_rotation(rotation)
        return self.turn_for(RIGHT, angle, DEGREES, settle_error=settle_error, timeout=timeout, wait=wait)

    def _turn_for(self, direction, angle, unit, settle_error, timeout):
        '''
        ### INTERNAL
        '''
        if unit is not RotationUnits.DEG: raise NotImplementedError("Units must be MM")
        self._command_running = True
        timer = Timer()

        turn_pid = PID(self.turn_pid_constants.Kp, self.turn_pid_constants.Ki, self.turn_pid_constants.Kd)
        turn_pid.set_output_limit(self.turn_pid_constants.max_output) # limit output to defined power
        turn_pid.set_output_ramp_limit(self.turn_pid_constants.max_ramp)
        # allow for per call settle_threshold and timeout, useful if we need to vary accuracy particularly when chaining motions
        turn_pid.set_settle_threshold(self.turn_pid_constants.settle_error if settle_error is None else settle_error) # settle threshold in degrees
        self._this_timeout = self.default_timeout if timeout is None else timeout
        turn_pid.set_timeout(self._this_timeout)
        start_rotation = self.inertial.rotation()
        target_rotation = start_rotation + (angle if direction == TurnType.RIGHT else -angle)
        loop_count = 0
        while not turn_pid.is_done() and not self._cancel_command:
            start_time = timer.time(SECONDS)

            current_rotation = self.inertial.rotation()
            pid_output = turn_pid.compute(target_rotation, current_rotation)

            self._spin(pid_output, -pid_output)

            loop_count += 1
            end_time = timer.time(SECONDS)
            wait(turn_pid.timestep - (end_time - start_time), SECONDS)

        self._was_timeout = turn_pid.get_is_timed_out()
        self._stop(self.stop_mode)
        rate = timer.time() / loop_count if loop_count > 0 else 0.0
        print("Done Turn: ", rate, timer.time(), turn_pid.get_is_settled(), turn_pid.get_is_timed_out())

        # for log_entry in turn_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)
        self._this_timeout = None
        self._command_running = False
        if (self._was_timeout): self._timeout_notifier.broadcast()
        return timer.time()
    
    def _turn_for_thread(self, args1, arg2, arg3, arg4, arg5):
        '''
        ### INTERNAL
        '''
        self._turn_for(args1, arg2, arg3, arg4, arg5)
    
    def turn_for(self, direction, angle, unit, settle_error = None, timeout = None, wait = True):
        self._concurrency_check()
        self._command_running = True
        if wait:
            return self._turn_for(direction, angle, unit, settle_error, timeout)
        else:
            self._worker_thread = Thread(self._turn_for_thread, (direction, angle, unit, settle_error, timeout))
            return 0

    def _drive_for(self, direction, distance, unit, heading, settle_error, timeout):
        '''
        ### INTERNAL
        '''
        if unit is not DistanceUnits.MM: raise NotImplementedError("Units must be MM")
        self._command_running = True
        timer = Timer()

        drive_pid = PID(self.drive_pid_constants.Kp, self.drive_pid_constants.Ki, self.drive_pid_constants.Kd)
        drive_pid.set_output_limit(self.drive_pid_constants.max_output) # limit output to 50% power
        drive_pid.set_output_ramp_limit(self.drive_pid_constants.max_ramp)
        # see if we want to override settle and timeout
        if (settle_error is None): drive_pid.set_settle_threshold(self.drive_pid_constants.settle_error)
        else: drive_pid.set_settle_threshold(self.calc_drive_settle_error(settle_error))
        self._this_timeout = self.default_timeout if timeout is None else timeout
        drive_pid.set_timeout(self._this_timeout)

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

        loop_count = 0
        while not drive_pid.is_done() and not self._cancel_command:
            start_time = timer.time(SECONDS)

            current_position = (
                (self.left_motors.position(RotationUnits.DEG) - left_start_pos) +
                (self.right_motors.position(RotationUnits.DEG) - right_start_pos)) / 2.0

            pid_output = drive_pid.compute(target_position, current_position)

            turn_pid_output = 0.0
            drive_turn_scaling = 1.0
            if (heading is not None):
                current_rotation = self.inertial.rotation()
                turn_pid_output = turn_pid.compute(target_rotation, current_rotation)
                
                if abs(pid_output) + abs(turn_pid_output) > 1.0:
                    drive_turn_scaling = (1.0 - abs(turn_pid_output)) / abs(pid_output)
                else:
                    drive_turn_scaling = cos(radians(target_rotation - current_rotation))

            pid_output *= drive_turn_scaling # reduce drive power when heading error is large
            self._spin(pid_output + turn_pid_output, pid_output - turn_pid_output)

            end_time = timer.time(SECONDS)
            wait_time = drive_pid.timestep - (end_time - start_time)
            loop_count += 1
            wait(wait_time, SECONDS)

        self._was_timeout = drive_pid.get_is_timed_out()
        self._stop(self.stop_mode)
        rate = timer.time() / loop_count if loop_count > 0 else 0.0
        print("Done Drive: ", rate, timer.time(), drive_pid.get_is_settled(), drive_pid.get_is_timed_out())

        # for log_entry in drive_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)
        self._this_timeout = None
        self._command_running = False
        if (self._was_timeout): self._timeout_notifier.broadcast()
        return timer.time()

    def _drive_for_thread(self, args1, arg2, arg3, arg4, arg5, arg6):
        '''
        ### INTERNAL
        '''
        self._drive_for(args1, arg2, arg3, arg4, arg5, arg6)
    
    def drive_for(self, direction, distance, unit, heading = None, settle_error = None, timeout = None, wait = True):
        self._concurrency_check()
        self._command_running = True
        if wait:
            return self._drive_for(direction, distance, unit, heading, settle_error, timeout)
        else:
            self._worker_thread = Thread(self._drive_for_thread, (direction, distance, unit, heading, settle_error, timeout))
            return 0

    def _drive_to_point(self, x, y, direction, orientation_callback, settle_error, timeout):
        '''
        ### INTERNAL
        '''
        self._command_running = True
        timer = Timer()

        drive_pid = PID(self.drive_pid_constants.Kp, self.drive_pid_constants.Ki, self.drive_pid_constants.Kd)
        drive_pid.set_output_limit(self.drive_pid_constants.max_output) # limit output to 50% power
        drive_pid.set_output_ramp_limit(self.drive_pid_constants.max_ramp)
        # see if we want to override settle and timeout
        if settle_error is None: drive_settle_error = self.drive_pid_constants.settle_error
        else: drive_settle_error = self.calc_drive_settle_error(settle_error)
        drive_pid.set_settle_threshold(drive_settle_error)
        self._this_timeout = self.default_timeout if timeout is None else timeout
        drive_pid.set_timeout(self._this_timeout)

        turn_pid = PID(self.heading_lock_pid_constants.Kp, self.heading_lock_pid_constants.Ki, self.heading_lock_pid_constants.Kd)
        turn_pid.set_output_limit(self.heading_lock_pid_constants.max_output) # limit output to 50% power
        turn_pid.set_settle_time(0.0)
        turn_pid.set_timeout(0.0)

        cur_x, cur_y, cur_heading = orientation_callback()
        start_angle = degrees(atan2(y - cur_y, x - cur_x))
        start_angle = start_angle if direction == DirectionType.FORWARD else InertialWrapper.to_angle(start_angle + 180.0)
        print("Start Drive to Point: ({:.1f}, {:.1f}), Direction: {}, Start Angle: {:.2f}".format(x, y, direction, start_angle))

        line_settled = False
        prev_line_settled = self.test_finish_line(x, y, start_angle, cur_x, cur_y)

        while not drive_pid.is_done() and not self._cancel_command:
            start_time = timer.time(SECONDS)

            cur_x, cur_y, cur_heading = orientation_callback()
            
            line_settled = self.test_finish_line(x, y, start_angle, cur_x, cur_y)
            if (line_settled and not prev_line_settled):
                print("Finished line to point at position: ({:.1f}, {:.1f})".format(cur_x, cur_y))
                break
            prev_line_settled = line_settled

            distance_error = ((x - cur_x) ** 2 + (y - cur_y) ** 2) ** 0.5
            distance_error_revs = 360.0 * distance_error / (self.wheel_travel_mm * self.ext_gear_ratio) # convert mm to wheel revolutions
            distance_error_revs = distance_error_revs if direction == DirectionType.FORWARD else -distance_error_revs

            target_heading = InertialWrapper.to_heading(degrees(atan2(y - cur_y, x - cur_x)))
            target_heading = target_heading if direction == DirectionType.FORWARD else InertialWrapper.to_heading(target_heading + 180.0)
            target_rotation = self.inertial.calc_rotation_at_heading(target_heading)
            current_rotation = self.inertial.rotation()

            # print("Drive to Point: Pos ({:.1f}, {:.1f}), Dist Err: {:.1f} mm, Heading Err: {:.2f} deg".format(
            #     cur_x, cur_y, distance_error, target_rotation - current_rotation))

            pid_output = drive_pid.compute(0.0, -distance_error_revs)

            turn_pid_output = 0.0
            drive_turn_scaling = 1.0
            if (distance_error_revs > drive_settle_error):
                turn_pid_output = turn_pid.compute(target_rotation, current_rotation)
            
            drive_turn_scaling = cos(radians(target_rotation - current_rotation))
            if drive_turn_scaling < 0.0: drive_turn_scaling = 0.0

            if abs(pid_output * drive_turn_scaling) + abs(turn_pid_output) > 1.0:
                drive_turn_scaling = (1.0 - abs(turn_pid_output)) / abs(pid_output * drive_turn_scaling)

            pid_output *= drive_turn_scaling # reduce drive power when heading error is large
            self._spin(pid_output + turn_pid_output, pid_output - turn_pid_output)

            end_time = timer.time(SECONDS)
            wait_time = drive_pid.timestep - (end_time - start_time)
            wait(wait_time, SECONDS)

        self._was_timeout = drive_pid.get_is_timed_out()
        self._stop(self.stop_mode)
        print("Done Drive: ", timer.time(), drive_pid.get_is_settled(), drive_pid.get_is_timed_out())

        # for log_entry in drive_pid.log:
        #     print(log_entry[0], ",", log_entry[1], ",", log_entry[2])
        #     wait(50, MSEC)
        self._this_timeout = None
        self._command_running = False
        if (self._was_timeout): self._timeout_notifier.broadcast()
        return timer.time()
        
    def _drive_to_point_thread(self, x, y, direction, orientation_callback, settle_error, timeout):
        '''
        ### INTERNAL
        '''
        self._drive_to_point(x, y, direction, orientation_callback, settle_error, timeout)

    def drive_to_point(self, x: float, y: float, direction, orientation_callback: Callable, settle_error: float | None = None, timeout: float | None = None, wait: bool = True):
        '''
        Docstring for drive_to_point
        
        :param x: Description
        :type x: float
        :param y: Description
        :type y: float
        :param direction: Description
        :param orientation_callback: Description
        :type orientation_callback: Callable
        :param settle_error: Description
        :type settle_error: float | None
        :param timeout: Description
        :type timeout: float | None
        :param wait: Description
        :type wait: bool
        '''
        self._concurrency_check()
        self._command_running = True
        if wait:
            return self._drive_to_point(x, y, direction, orientation_callback, settle_error, timeout)
        else:
            self._worker_thread = Thread(self._drive_to_point_thread, (x, y, direction, orientation_callback, settle_error, timeout))
            return 0


    def _spin(self, left_speed, right_speed):
        '''
        ### INTERNAL

        ### Arguments
            left_speed ([-1.0, 1.0])
            right_speed ([-1.0, 1.0])
        '''
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
        '''
        ### Spins the motors

        Motors will keep spinning at the commanded speed after the call returns

        Will generate an exception if there is a currently running command

        ### Arguments
            left_speed (PERCENT, [-100, 100]): Speed for left motor group
            right_speed (PERCENT, [-100, 100]): Speed for right motor group

        ### Returns
            None
        '''
        self._concurrency_check()
        self._spin(left_speed / 100.0, right_speed / 100.0)
        return True

    def _stop(self, mode):
        '''
        ### INTERNAL
        '''
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
