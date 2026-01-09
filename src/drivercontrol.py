from vex import *
from inertialwrapper import InertialWrapper

class DriverControl:
    '''
    ### DriverControl class

    This class implements driver control for a tank drive robot using a left and right motor group and an inertial sensor.
    It includes features such as:
    - Deadband handling to prevent motor creep
    - Ramp control to limit acceleration/deceleration rates
    - Detwitching to reduce turn sensitivity at low speeds
    - Gyro-based straight driving assistance
    - Selctable brake modes

    :param left_motor_group: MotorGroup for the left side of the drivetrain
    :param right_motor_group: MotorGroup for the right side of the drivetrain
    :param inertial_sensor: InertialWrapper for the gyro sensor
    '''
    # Constants to convert percent to volt for drivetrain
    MOTOR_MAXVOLT = 11.5 # volts
    MOTOR_VOLTSCALE = MOTOR_MAXVOLT / 100.0

    # Constant for controller deadband - below this value treat motors as stopped. Avoids robot creep and potential chattering
    # Note that this is applied to the combined forward and rotate values from the controller, so
    MOTOR_DEADBAND = 5

    # Default maximum drive and turn rates
    DEFAULT_TURN_MAX = 66.0 # maximum turn rate
    DEFAULT_DRIVE_MAX = 100.0 # maximum drive rate
    # Default ramp control limit
    DEFAULT_MAX_CONTROL_RAMP = 5.0 # percent per timestep (assumed to be 10ms)

    # Default detwitch control
    DEFAULT_PIVOT_MAX_TURN_SPEED = 33.0
    DEFAULT_PIVOT_MIN_DRIVE_SPEED = 33.0
    DEFAULT_FULL_TURN_DRIVE_SPEED = 66.0
    
    def __init__(self, left_motor_group: MotorGroup, right_motor_group: MotorGroup, inertial_sensor: InertialWrapper):
        self.lmg = left_motor_group
        self.rmg = right_motor_group
        self.gyro = inertial_sensor

        # Configuration
        self.turn_max = DriverControl.DEFAULT_TURN_MAX
        self.drive_max = DriverControl.DEFAULT_DRIVE_MAX
        self.ramp_max = DriverControl.DEFAULT_MAX_CONTROL_RAMP
        self.pivot_max_turn = DriverControl.DEFAULT_PIVOT_MAX_TURN_SPEED  # maximum turn speed during pivot
        self.pivot_min_drive_speed = DriverControl.DEFAULT_PIVOT_MIN_DRIVE_SPEED # drive speed at which to start increasing turn rate
        self.full_turn_drive_speed = DriverControl.DEFAULT_FULL_TURN_DRIVE_SPEED # drive speed at which to use full turn rate

        # Enables
        self.enable_drive_straight = False
        self.enable_heading_lock = False
        self.enable_percent_drive = False
        self.enable_brake_mode = False
        self.enable_ramp_control = True
        self.enable_detwitch = True
    
        # Ramp control variables - assumes robot is stationary at start
        self.last_speed = 0
        self.last_turn = 0

        # Motor control variables - 
        #  - drivetrain_running says if drivetrain motors can spin. If false, the controller is in the deadband range and we stop the motors 
        self.drivetrain_running = False

        self.follow_heading = None
        self.follow_heading_Kp = 2.0

    def drivetrain_detwitch(self, speed, turn, enabled):
        '''
        ### (INTERNAL) )ETWITCH - reduce turn sensitiviy when robot is moving slowly (turning in place)

        NOTE: speed is not altered only turn

        :param speed: speed in percent - from -100 to +100 (full reverse to full forward)
        :param turn: turn in percent - from -100 to +100 (full left turn to full right turn)
        :param enabled: indicates whether to enable the detwitch code or not

        :return: speed (unmodified) and turn based on simple straight line segments
        '''

        if not enabled:
            return speed * self.drive_max / 100.0, turn * self.turn_max / 100.0

        # Region 1: below minimum drive speed - use minimum turn rate
        turn_scale = self.pivot_max_turn / 100.0 # start off with minimum turn rate
        # Region 2: between minimum drive speed and full turn drive speed - linearly increase turn rate
        if (abs(speed) >= self.pivot_min_drive_speed and abs(speed) < self.full_turn_drive_speed):
            # linearly increase the turn rate between the drive speed setpoints using a straight line equation
            #  y = a + b * x
            a = self.pivot_max_turn / 100.0
            b = ((self.turn_max - self.pivot_max_turn) / 100.0) / (self.full_turn_drive_speed - self.pivot_min_drive_speed)
            turn_scale = a + b * (abs(speed) - self.pivot_min_drive_speed)
        # Region 3: above full turn drive speed - use full turn rate
        elif (abs(speed) >= self.full_turn_drive_speed):
            turn_scale = self.turn_max / 100.0

        turn = turn * turn_scale
        speed = speed * self.drive_max / 100.0

        return speed, turn

    def drivetrain_ramp_limit(self, speed, turn, enabled):
        '''
        ###  (INTERNAL) RAMP LIMIT - limit how fast we can go from one extreme to another on the joysticks

        The max range is 200 percent (ie from -100 to +100). A value of 20 for MAX_CONTROL_RAMP would take 0.1s to go from full
        forward to full reverse, or from full left to full right turn

        NOTE: This is done on the control inputs to avoid potential motion artifacts if done on motors separately

        :param speed: percent forward/reverse speed
        :param turn: percent left/right speed
        :param enabled: indicates wether to perform the ramp limit or not

        :return: ramp controlled speed and turn (in percent)
        '''
        if (enabled):
            if (abs(speed - self.last_speed) > self.ramp_max):
                if (speed > self.last_speed): speed = self.last_speed + self.ramp_max
                else: speed = self.last_speed - self.ramp_max

            if (abs(turn - self.last_turn) > self.ramp_max):
                if (turn > self.last_turn): turn = self.last_turn + self.ramp_max
                else: turn = self.last_turn - self.ramp_max

        self.last_speed = speed
        self.last_turn = turn

        return speed, turn
    

    def clamp(self, input, clamp_value = 100.0):
        '''
        ###  (INTERNAL) CLAMPING - limits output to range -clamp_value, clamp_value
        
        :param input: input in percent
        :param clamp_value (optional): clamp_value - defaults to 100 percent

        :return: clamped value (in percent)
        '''
        return max(min(input, clamp_value), -clamp_value)
    
    # CONTROLLER_DEADBAND - used in case controller has some drift, mostly for turning
    def controller_deadband(self, input, deadband, max_range = 100.0):
        '''
        ###  (INTERNAL) Docstring for controller_deadband
        
        :param self: Description
        :param input: Description
        :param deadband: Description
        :param max_range: Description
        '''
        output = 0.0
        scale = max_range / (max_range - deadband)
        if (abs(input) < deadband):
            output = 0.0
        elif (input > 0.0):
            output = (input - deadband) * scale
        elif (input < 0.0):
            output = (input + deadband) * scale

        return output
    
    # GYRO_ROTATION - get the current rotation value in degrees and scale by the gyro scale
    def gyro_rotation(self):
        '''
        ### (INTERNAL) Docstring for gyro_rotation
        
        :param self: Description
        '''
        return self.gyro.rotation()

    # DRIVE_STRAIGHT - will attempt to follow gyro heading when enabled
    def drive_straight(self, speed):
        '''
        ### (INTERNAL) Docstring for drive_straight
        
        :param self: Description
        :param speed: Description
        '''
        if (self.gyro is None or not self.gyro.installed):
            return speed, 0.0
        
        if (self.follow_heading is None):
            self.follow_heading = self.gyro_rotation()
            # print("Follow enabled", self.follow_heading)
            return speed, 0.0

        error = self.follow_heading - self.gyro_rotation()
        Kp = self.follow_heading_Kp
        turn = error * Kp

        # Note: motors turn 10 revolutions per robot 360deg rotation, or 10 * 24 / 60 = 4 wheel rotations
        # - Wheels travel a distance of 4 * 260mm = 1.04m
        # - To turn robot 360deg in one second is 10 revs / sec or 600RPM (coincidently full speed of the blue motors)
        # - Therefore one percent of turn command corresponds to 3.6deg/sec of robot rotational velocity
        # - With Kp of 4 it means we are commanding robot to turn at 14.4deg/sec per degree of heading error

        return speed, turn
    
    # CANCEL_DRIVE_STRAIGHT - resets the heading
    def cancel_drive_straight(self):
        '''
        ### (INTERNAL) Docstring for cancel_drive_straight
        
        :param self: Description
        '''
        if (not self.enable_heading_lock and self.follow_heading is not None):
            # print("Follow cancelled")
            self.follow_heading = None

    def set_mode(self,
                 enable_drive_straight = None,
                 enable_heading_lock = None,
                 enable_percent_drive = None,
                 enable_brake_mode = None,
                 enable_ramp_control = None,
                 enable_detwitch = None,
                 follow_heading = None,
                 follow_heading_Kp = None):
        '''
        ### Docstring for set_mode
        
        :param enable_drive_straight: Description
        :param enable_heading_lock: Description
        :param enable_percent_drive: Description
        :param enable_brake_mode: Description
        :param enable_ramp_control: Description
        :param enable_detwitch: Description
        :param follow_heading: Description
        :param follow_heading_Kp: Description
        '''
        
        if enable_drive_straight is not None: self.enable_drive_straight = enable_drive_straight
        if enable_heading_lock is not None: self.enable_heading_lock = enable_heading_lock
        if enable_percent_drive is not None: self.enable_percent_drive = enable_percent_drive
        if enable_brake_mode is not None: self.enable_brake_mode = enable_brake_mode
        if enable_ramp_control is not None: self.enable_ramp_control = enable_ramp_control
        if enable_detwitch is not None: self.enable_detwitch = enable_detwitch

        if enable_heading_lock is not None:
            if enable_heading_lock:
                self.follow_heading = follow_heading
            else:
                self.follow_heading = None

        if follow_heading_Kp is not None: self.follow_heading_Kp = follow_heading_Kp

    def set_speed_limits(self, drive_max = None, turn_max = None, ramp_max = None):
        '''
        ### Docstring for set_speed_limits
        
        :param drive_max: Description
        :param turn_max: Description
        :param ramp_max: Description
        '''
        if drive_max is not None: self.drive_max = drive_max
        if turn_max is not None: self.turn_max = turn_max
        if ramp_max is not None: self.ramp_max = ramp_max

    def set_detwitch_params(self, pivot_max_turn = None, pivot_min_drive_speed = None, full_turn_drive_speed = None):
        '''
        ### Docstring for set_detwitch_params
        
        :param pivot_max_turn: Description
        :param pivot_min_drive_speed: Description
        :param full_turn_drive_speed: Description
        '''
        if pivot_max_turn is not None: self.pivot_max_turn = pivot_max_turn
        if pivot_min_drive_speed is not None: self.pivot_min_drive_speed = pivot_min_drive_speed
        if full_turn_drive_speed is not None: self.full_turn_drive_speed = full_turn_drive_speed

    def user_drivetrain(self, control_speed, slow_turn_axis=0.0, fast_turn_axis=0.0):
        '''
        ### USER DRIVETRAIN - main entry for user control. Should be called every 10ms
        
        First calls the detwitch function\\
        Secondly calls the ramp control\\
        Thirdly checks deadband range and instructs motors to either run or stop
        
        The deadband logic may seem a bit convoluted, but it prevents the motor from being "stopped" every cycle
         - drivetrain_running is used as a flag so we only stop once until the controls move above the deadband again

        :param control_speed: is raw controller forward / backwards speed in percent
        :param control_slow_turn: is raw controller left / right speed in percent for the slow axis (this overrides fast axis)
        :param control_fast_turn: is raw controller left / right speed in percent for the fast axis

        :returns: No return value
        '''
        # calculate the drivetrain motor velocities from the controller joystick axes

        # just in case - make sure there is no turn coming from the joystick unless we want it
        control_slow_turn = self.controller_deadband(slow_turn_axis, 4.0)
        control_fast_turn = self.controller_deadband(fast_turn_axis, 10.0)
        # deadband logic will keep output at zero until deadband exceeded
        if (control_slow_turn != 0.0):
            control_turn = control_slow_turn
            detwitch = self.enable_detwitch
        else:
            control_turn = control_fast_turn
            detwitch = False

        # Select auto follow heading mode if enabled and we are not commanded to turn, and are not waiting on a turn to finish
        # TODO: add case for when coasting to stop, but ramp control is still active
        if (self.enable_heading_lock or (self.enable_drive_straight and (control_speed != 0.0 or self.last_speed != 0.0))) and control_turn == 0.0 and self.last_turn == 0.0:
            detwitch_speed = control_speed * self.drive_max / 100.0
            auto_speed, auto_turn = self.drive_straight(detwitch_speed)
            safe_speed, _ = self.drivetrain_ramp_limit(auto_speed, 0, self.enable_ramp_control)
            safe_turn = auto_turn
        # Else just follow along with what the driver is doing
        else:
            self.cancel_drive_straight()
            detwitch_speed, detwitch_turn = self.drivetrain_detwitch(control_speed, control_turn, detwitch)
            safe_speed, safe_turn = self.drivetrain_ramp_limit(detwitch_speed, detwitch_turn, self.enable_ramp_control)
        
        # mix together speed and turn and clamp combined values to -100 to +100 percent
        drivetrain_left_side_speed = self.clamp(safe_speed + safe_turn)
        drivetrain_right_side_speed = self.clamp(safe_speed - safe_turn)

        # check if the values are inside of the deadband range
        if abs(drivetrain_left_side_speed) < DriverControl.MOTOR_DEADBAND and abs(drivetrain_right_side_speed) < DriverControl.MOTOR_DEADBAND:
            # check if the motors have already been stopped
            if self.drivetrain_running:
                # stop the drive motors
                if self.enable_brake_mode:
                    self.lmg.stop(BRAKE)
                    self.rmg.stop(BRAKE)
                else:
                    self.lmg.stop(COAST)
                    self.rmg.stop(COAST)

                # tell the code that the motors have been stopped
                self.drivetrain_running = False
        else:
        # reset the toggle so that the deadband code knows to stop the motors next
        # time the input is in the deadband range
            self.drivetrain_running = True

        # NOTE: supplying VOLT shows as a mismatched type error, although it runs fine as is valid per the API. 'type: ignore' silences the error
        # only tell the left drive motor to spin if the values are not in the deadband range
        if self.drivetrain_running:
            if self.enable_percent_drive:
                self.lmg.spin(FORWARD, drivetrain_left_side_speed, PERCENT)
            else:
                self.lmg.spin(FORWARD, drivetrain_left_side_speed * DriverControl.MOTOR_VOLTSCALE, VOLT) # type: ignore

        # only tell the right drive motor to spin if the values are not in the deadband range
        if self.drivetrain_running:
            if self.enable_percent_drive:
                self.rmg.spin(FORWARD, drivetrain_right_side_speed, PERCENT)
            else:
                self.rmg.spin(FORWARD, drivetrain_right_side_speed * DriverControl.MOTOR_VOLTSCALE, VOLT) # type: ignore

