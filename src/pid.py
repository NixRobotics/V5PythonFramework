class PID:
    '''
    ### "Simple" PID controller class for demonstration purposes only

    This provides the basic functionality required by most controllers including feedforward

    The output range should be in the range [-1.0, 1.0]

    Input scaling is not performed, so to not have very large or small K values pick an input range that makes sense, e.g. degrees
    works well for turning with a Kp of around 1.0. Similarly using wheel revolutions in degrees for tracking distance does the same
    thing with a drive Kp of 1.0 being a good starting point

    K values are also scaled by the timestep to help get Kp values roughly in the ballpark

    Some required features implemnted include:
    - Integral wind-up protection using backcalculation of the saturation limit of the controller, and also zero crossing detection
      Note zero crossing is not generally recommended, but it helps make the code simpler
    - Programmable settling time, threshold and timeout values. Setting these to 0.0 will let controller run indefinitely (e.g. for heading lock)
    - Output and ramp limits allow for controlling max output swing as well as acceleration

    Not implemented:
    - Proper initialization of last values in the case that the inputs are not zero at the start
    - Resetting state. Its assumed the class is created for each separate command needing PID
    - Deadband control. In the case where the robot needs a minimum power to get it to move, outputs below this obviously won't achieve anything
    '''

    def __init__(self, Kp, Ki, Kd, Kf = 0.0):
        self.timestep = 0.01 # approximate timestep in seconds - used to process timeouts. Changing this will scale the K values
        
        # Constants
        self.Kp = Kp * self.timestep
        self.Ki = Ki * self.timestep
        self.Kd = Kd * self.timestep
        self.Kf = Kf

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

        # Time parameters
        self.settle_timer_limit = 0.1 # default settle time in seconds, set to 0.0 to disable settling
        self.settle_error_threshold = 1.0 # unit agnostic settle threshold
        self.timeout_timer_limit = 10.0 # default timeout in seconds, set to 0.0 to disable timeout

        # Timers
        # For free running PID such as heading lock disable these by setting time limits above to 0.0
        self.settle_timer = self.settle_timer_limit
        self.timeout_timer = self.timeout_timer_limit

        # Output limits and integral windup controls
        self.output_limit = 1.0 # default output limit (actual output will be clamped to -output_limit, +output_limit)
        self.output_ramp_limit = 0.0 # default output ramp limit (max change in output per compute() call). 0.0 = no ramp limit
        self.integral_limit = self.output_limit / self.Kp # default integral limit (integral term will be clamped to -integral_limit, +integral_limit)

        # Output flags
        self.is_timed_out = False
        self.is_settled = False

        # self.log = []

    # Setter functions
    def set_settle_time(self, time_sec):
        self.settle_timer_limit = time_sec

    def set_timeout(self, time_sec):
        self.timeout_timer_limit = time_sec
        self.timeout_timer = time_sec

    def set_settle_threshold(self, threshold):
        '''
        Our settle threshold will be in our measurement units. If degrees, we want this to be about 0.5degrees or less
        '''
        self.settle_error_threshold = threshold

    def set_timestep(self, timestep_sec):
        self.Ki *= timestep_sec / self.timestep
        self.Kd /= timestep_sec / self.timestep
        self.Kp /= timestep_sec / self.timestep
        self.timestep = timestep_sec

    def set_output_limit(self, limit):
        '''
        Output should be normalized to range [-1.0, +1.0]
        '''
        self.output_limit = limit
        self.set_integral_limit(self.output_limit / self.Kp)

    def set_output_ramp_limit(self, ramp_limit):
        '''
        Ramp limit will be based on our normaled output range and timestep

        So for a timestep of 0.01sec and a ramp limit of 0.1, the output can change by a maximum of 0.1 every 0.01sec

        If output_limit is set to 1.0 this means it will take at least 0.2sec to go from -1.0 to +1.0 output
        '''
        self.output_ramp_limit = ramp_limit


    def set_integral_limit(self, limit):
        '''
        Integral limit will be based on our units of measurement and take the output_limit into consideration

        E.g. for a heading controller we will want to limit the output to the range [-output_limit, +output_limit] which will
        represent how fast we want each side to turn in the default range of [-1.0, +1.0] (1.0 = full power or 100%)

        If the heading error is large enough we just want the maximum output and do not want the integral term to accumlate

        The point at which we fall under the output limit will be when we transition from the saturation region to the controlable
        region. This will typically be around an error value of around 10 degrees but will depend on the Kp value needed, so we
        can only set this once we have some idea of what Kp will be.

        Ideally our resulting error should represent some physically meaningful value so error in this case would be in degrees
        - E.g. if we set output_limit to 0.5 (50% power) and Kp to 0.01 we would saturate the output down to an error of 50deg
        - saturation point = output_limit = Kp * error  => error = output_limit / Kp or 0.5 % / 0.01 Kp = 50 degrees
        - Therefore our integral limit would be set to 50 (degrees) in this case
        '''
        self.integral_limit = limit

    # Getter functions
    def get_is_timed_out(self):
        return self.is_timed_out
    
    def get_is_settled(self):
        return self.is_settled

    # Main compute function
    def compute(self, setpoint, measurement):
        if self.is_done(): return 0.0

        error = setpoint - measurement

        derivative = error - self.prev_error

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative) + (self.Kf * setpoint)

        # Output limiting
        is_output_limited, output = self.limit(output, self.output_limit)
        
        # Output ramp limiting
        is_ramp_limited = False
        if (self.output_ramp_limit > 0.0):
            is_ramp_limited, output = self.ramp_limit(output, self.prev_output, self.output_ramp_limit)

        self.prev_output = output

        # Integral windup control
        # Case 1: only accumulate integral if error is less than saturation limit. We reset to zero to allow for changing setpoints
        if abs(error) > self.integral_limit:
            self.integral = error
        # Case 2: reset integral if error crosses zero
        elif (error > 0.0 and self.prev_error < 0.0) or (error < 0.0 and self.prev_error > 0.0):
            self.integral = 0.0
        elif (is_output_limited or is_ramp_limited):
            self.integral = error
        else:
            self.integral += error

        self.prev_error = error

        # TODO: minimum output for small errors

        # Timeouts and settling
        if abs(error) < self.settle_error_threshold and not is_ramp_limited:
            self.settle_timer -= self.timestep
        else:
            self.settle_time = self.settle_timer_limit
        self.timeout_timer -= self.timestep

        if (self.is_done()): return 0.0

        # self.log.append([error, output, self.integral])

        return output
    
    # Timeout and settle check function
    def is_done(self):
        if (self.settle_timer_limit > 0.0 and self.settle_error_threshold > 0.0 and self.settle_timer <= 0.0):
            self.is_settled = True
            return True
        
        if self.timeout_timer_limit > 0.0 and self.timeout_timer <= 0.0:
            self.is_timed_out = True
            return True
        
        return False
    
    # Output limiting
    def limit(self, value, limit):
        limited_output = value
        is_limited = False
        if value > limit:
            limited_output = limit
        elif value < -limit:
            limited_output = -limit
        if (limited_output != value): is_limited = True

        return is_limited, limited_output
    
    def ramp_limit(self, value, prev_value, limit):
        ramp_limited_output = value
        is_ramp_limited = False
        if (abs(value - prev_value) > limit):
            # if (value > 0.0 and value > prev_value): ramp_limited_output = prev_value + limit
            # elif (value < 0.0 and value < prev_value): ramp_limited_output = prev_value - limit
            if (value > prev_value): ramp_limited_output = prev_value + limit
            elif (value < prev_value): ramp_limited_output = prev_value - limit
        if (value != ramp_limited_output): is_ramp_limited = True

        return is_ramp_limited, ramp_limited_output