# Library imports
from vex import *
from math import sin, cos, radians, degrees, atan2, sqrt
from collections import namedtuple
from inertialwrapper import InertialWrapper

class Tracking:
    '''
    ### Tracking class to calculate robot position using tracking wheels or motor encoders plus inertial sensor

    Internally everything is based in meters and radians
    To distinguish from VEX uses for various heading readings, we use the following terminology:
    - ROTATION: continuous rotation value in degrees (can be positive or negative, no bounds)
    - HEADING: bounded heading in degrees [0, 360)
    - ANGLE: bounded angle in degrees (-180, +180]
    - THETA: continuous rotation value in radians (can be positive or negative, no bounds). Same as ROTATION but in radians
    THETA is used internally and converted to/from HEADING/ANGLE as needed
    
    Note that  __init__() and update_location() assume that the gyro scale factor has already been applied to the inertial sensor readings

    :param devices: List of 3 devices in this order: [left motor group, right motor group, inertial] or [fwd tracker, side tracker, inertial]
    :param (optional) orientation: Starting orientation consisting of an orientation tuple (x in mm, y in mm, heading in degrees)
    :param (optional) configuration: Descriptor for tracking configuration. See set_configuration() for more details
    '''
    
    Orientation = namedtuple('Orientation', ['x', 'y', 'heading'])
    
    # Configuration initializers
    # @param *_wheel_size is the the size of the odometry wheel. Set to 0.0 if not present. If only one forward wheels is present, use left regardless
    #  if it is mounted on the left of the right of the robot
    # @param *_gear_ratio is any gear ratio used
    # @param *_offset is the offset of the tracking wheel relative to the turning center of the robot. Positive left/right is to RIGHT of robot, so
    #  wheels mounted on left of robot would be negative. Positive side is to FRONT of robot, so wheels mounted towards the back of the robot
    #  would be negative
    # @param fwd_is_odom. If False left and rigght wheels are trated as the motor left and right motor group encoders
    # It is assumed that encoders are configured correctly such that FORWARD motion is positive for both left and right encoders and RIGHT motiion
    # is positive to side/strafe encoder

    class Configuration:
        def __init__(self,
            fwd_is_odom,
            fwd_wheel_size, fwd_gear_ratio, fwd_offset,
            side_wheel_size, side_gear_ratio, side_offset):

            self.fwd_is_odom = fwd_is_odom
            self.fwd_wheel_size = fwd_wheel_size
            self.fwd_gear_ratio = fwd_gear_ratio
            self.fwd_offset = fwd_offset
            self.side_wheel_size = side_wheel_size
            self.side_gear_ratio = side_gear_ratio
            self.side_offset = side_offset
    
    # Encoder initializers
    # @param left is initial left encoder position in revolutions (either left motors or left odom wheek)
    # @param right is initial right encoder position in revolutions (either right motors or right odom wheel)
    # @param side is sideways or strafe encoder in revolutions (only valid if sideways odom wheel installed)
    # @param theta is initial gyro theta in radians (currently not used - the inertial sensor will be programmed to the initial heading)
    # This assumes that when declaring devices that robot forward is positive and robot right is positive. Positive rotation is towards the right
    EncoderValues = namedtuple('EncoderValues', ['left', 'right', 'side', 'theta'])

    # Tracking wheel geometry
    # In this case we are just using motor encoders and gyro, however same concept works for odometry wheels
    DEFAULT_GEAR_RATIO = 60.0 / 60.0 # external gear ratio
    DEFAULT_WHEEL_SIZE = 320.0 # mm
    # FWD_OFFSET is the distance from the robot center to the forward tracking wheel, right is positive
    DEFAULT_FWD_OFFSET = 0.0 # mm
    # SIDE_OFFSET is the distance from the robot center to the side tracking wheel, forward is positive
    DEFAULT_SIDE_OFFSET = 0.0 # mm

    def __init__(self,
                 devices,
                 orientation: Union[Orientation, None] = None,
                 configuration: Union[Configuration, None] = None,
                 name: str = "Tracker"
                 ):

        self._name = name
        print('init', self._name)

        self._is_enabled = False
        self._enable_resampling = True

        x = 0.0 if orientation is None else orientation.x
        y = 0.0 if orientation is None else orientation.y
        heading = 0.0 if orientation is None else orientation.heading

        self.x = x # MM NORTH
        self.y = y # MM EAST

        # heading passed in as degrees 0 to 360. Converted to continuous radians
        # theta is our internal (continous) rotation in radians 
        # theta reflects the true rotation of the robot not the uncorrected gyro version
        # We also set reset the gyro reading to match our heading. The set_sensor_heading() call will apply the gyro scaling factor
        self.theta = InertialWrapper.to_angle(radians(heading))
        self.inertial = devices[2]
        if self.inertial is None:
            raise Exception("ERROR: INERTIAL SENSOR MUST BE PRESENT ON FIRST INITIALIZATION")
        self._set_sensor_heading(heading)

        # Configuration
        self.fwd_is_odom = False
        self.fwd_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.fwd_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.fwd_offset = Tracking.DEFAULT_FWD_OFFSET

        self.side_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.side_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.side_offset = Tracking.DEFAULT_SIDE_OFFSET
        if (configuration is not None): self.set_configuration(configuration)

        self.timer = Timer()
        self.avg_time = 0
        self.avg_rate = 0
        self.timestep = 5 # ms

        # Pervious values for odometry calculations
        self.previous_left_position = 0.0 # revolutions
        self.previous_right_position = 0.0 # revolutions
        self.previous_side_position =  0.0 # revolutions
        self.previous_theta = self.theta # radians

        # Keep track of raw encoder values for resampling
        self.latest_encoders = self.previous_encoders = [0.0, 0.0, 0.0, 0.0]
        self.latest_timestamps = self.previous_timestamps = [0, 0, 0, 0]

        self.this_thread = self.start_tracker(devices)

    @property
    def name(self):
        '''
        ### Docstring for name
        
        :returns: Name of tracker
        '''
        return self._name

    def _set_initial_values(self, initial_values: EncoderValues, initial_timestamps: EncoderValues):
        '''
        ### INTERNAL: Docstring for set_initial_values
        
        :param initial_values: Description
        :type initial_values: EncoderValues
        :param initial_timestamps: Description
        :type initial_timestamps: EncoderValues
        '''
        self.previous_left_position = initial_values.left # revolutions
        self.previous_right_position = initial_values.right # revolutions
        self.previous_side_position = initial_values.side # revolutions
        self.previous_theta = initial_values.theta # radians

        self._init_rolling_buffers(initial_values, initial_timestamps)

    def _init_rolling_buffers(self, initial_values: Union[EncoderValues, None], initial_timestamps: Union[EncoderValues, None], mask = None):
        '''
        ### INTERNAL: Docstring for init_rolling_buffers
        
        :param self: Description
        :param initial_values: Description
        :type initial_values: Union[EncoderValues, None]
        :param initial_timestamps: Description
        :type initial_timestamps: Union[EncoderValues, None]
        :param mask: Description
        '''
        if initial_values is None or initial_timestamps is None:
            raise RuntimeError("Rolling Buffer: Initial Values and Timestamps Can Not Be None")
        for i in range(len(initial_values)):
            if mask is None or mask == i:
                self.latest_encoders[i] = initial_values[i]
                self.latest_timestamps[i] = initial_timestamps[i]
                self.previous_encoders[i] =  self.latest_encoders[i]
                self.previous_timestamps[i] = self.latest_timestamps[i]

    def set_configuration(self, configuration: Configuration):
        '''
        ### Configuration initializers

        :param configuration.fwd_wheel_size: Circumference of the forward odometry wheel in mm. Set to 0.0 if not present. If only one forward wheel is present,\
            use left regardless if it is mounted on the left of the right of the robot. Motor tracking applies setting to both drivetrain sides
        :param configuration.side_sheel_size: Circumference of the sideways odometry wheel in mm. Set to 0.0 if not present
        :param configuration.fwd_gear_ratio: Is any gear ratio used between the wheel and the rotation sensor or motor
        :param configuration.side_gear_ratio: Is any gear ratio used between the wheel and the rotation sensor
        :param configuration.fwd_offset: Is the offset of the forward tracking wheel relative to the turning center of the robot. Positive is to RIGHT of robot, so\
            wheels mounted on left of robot would be negative
        :param configuration.side_offset: Is the offset of the sideways tracking wheel relative to the turning center of the robot. Positive side is to FRONT of robot,\
            so wheels mounted towards the back of the robot would be negative
        :param configuration.fwd_is_odom: If False left and right wheels are treated as the motor left and right MotorGroup encoders\
            It is assumed that encoders are configured correctly such that FORWARD motion is positive for both left and right encoders and RIGHT motiion\
            is positive to side/strafe encoder
        '''
        self.fwd_is_odom = configuration.fwd_is_odom
        self.fwd_wheel_size = configuration.fwd_wheel_size
        self.fwd_gear_ratio = configuration.fwd_gear_ratio
        self.fwd_offset = configuration.fwd_offset

        self.side_wheel_size = configuration.side_wheel_size
        self.side_gear_ratio = configuration.side_gear_ratio
        self.side_offset = configuration.side_offset
   
    def enable(self, enabled = True):
        '''
        ### Docstring for enable
        
        :param self: Description
        :param enabled: Description
        '''
        self._is_enabled = enabled

    def enable_resampling(self, enabled = True):
        '''
        ### Docstring for enable_resampling
        
        :param self: Description
        :param enabled: Description
        '''
        self._enable_resampling = enabled

    def current_heading(self):
        '''
        ### Gets current tracker heading in degrees (may differ from inertial sensor due to time lag)

        Theoretically this is same as calling GyroHelper.gyro_heading(), but it will vary due to sampling time of the sensor and accumulator effects

        :returns: Internal theta (radians) converted to degrees heading [0, 360)

        '''
        heading_deg = degrees(self.theta)
        return InertialWrapper.to_heading(heading_deg)

        # 0 is older, 1 is newer
    def _linear_interp(self, s0, s1, t0, t1, t_ref):
        '''
        ### INTERNAL Docstring for linear_interp
        
        :param self: Description
        :param s0: Description
        :param s1: Description
        :param t0: Description
        :param t1: Description
        :param t_ref: Description
        '''
        if t1 == t0: return s1
        if t0 == t_ref: return s0
        if t1 == t_ref: return s1
        s_new = s0 + (t_ref - t0) * ((s1 - s0) / (t1 - t0))
        return s_new
        
    def _rolling_buffer(self, values, timestamps, index):
        '''
        ### INTERNAL Docstring for rolling_buffer
        
        :param values: Description
        :param timestamps: Description
        :param index: Description
        :returns: True if updated, False if not
        '''
        if self.previous_encoders is None or self.latest_encoders is None:
            raise RuntimeError("Previous Encoder Values Not Set")
        
        if timestamps[index] > self.latest_timestamps[index]:
            self.previous_timestamps[index] = self.latest_timestamps[index]
            self.latest_timestamps[index] = timestamps[index]
            self.previous_encoders[index] = self.latest_encoders[index]
            self.latest_encoders[index] = values[index]
            return True
        
        return False

    def _resample(self, values, timestamps):
        '''
        ### INTERNAL Docstring for resample
        
        :param values: Description
        :param timestamps: Description
        '''
        # We attempt to align everything to the gyro, if that does not get updated we'll just use the latest values
        # gyro skips very infrequently so any anomolies from this should be minimal

        # Update rolling buffers
        updated = [False] * len(values)
        for i in range(len(values)):
            updated[i] = self._rolling_buffer(values, timestamps, i)

        return_values = [0.0] * len(values)
        reference_time_index = len(values) - 1
        reference_time_stamp = self.latest_timestamps[reference_time_index]
        for i in range(len(values) - 1):
            #if updated[i]: reference_time_stamp = self.latest_timestamps[i]
            #else: reference_time_stamp = self.latest_timestamps[i] + 0.01
            return_values[i] = self._linear_interp(
                self.previous_encoders[i],
                self.latest_encoders[i],
                self.previous_timestamps[i],
                self.latest_timestamps[i],
                reference_time_stamp)
        return_values[reference_time_index] = values[reference_time_index]

        return return_values

    def _calc_timestep_arc_chord(self, x, y, theta, delta_forward, delta_side, delta_theta):
        '''
        ### INTERNAL

        x, y, delta_forward, delta_side in MM

        theta, delta_theta in radians
        '''

        # local deltas
        if (delta_theta == 0.0):
            # no turn - use simple deltas
            delta_local_x = delta_forward
            delta_local_y = delta_side
            to_global_rotation_angle = theta
        else:
            # robot turning
            # calculate radius of movement for forward and side wheels
            r_linear = -self.fwd_offset + (delta_forward / delta_theta) # mm
            r_strafe = self.side_offset + (delta_side / delta_theta) # mm

            # calculate chord distances using chord length = 2 * r * sin(theta / 2)
            # pre-rotate by half the turn angle so we have only distance along one axis for each
            # when we rotate to global frame we need to account for this half-angle rotation
            to_global_rotation_angle = theta + delta_theta / 2
            delta_local_x = r_linear * 2.0 * sin(delta_theta / 2.0)
            delta_local_y = r_strafe * 2.0 * sin(delta_theta / 2.0)

        # rotate to global
        delta_global_x = delta_local_x * cos(to_global_rotation_angle) - delta_local_y * sin(to_global_rotation_angle)
        delta_global_y = delta_local_x * sin(to_global_rotation_angle) + delta_local_y * cos(to_global_rotation_angle)

        return (x + delta_global_x, y + delta_global_y, theta + delta_theta)

    def _update_location(self, values, timestamps):
        '''
        ### INTERNAL

        ### Arguments
            values: array of 4 values: left(REV), right(REV), side(REV), theta(RADIANS)
            timestamps: array of 4 timestamps: left(MS), right(MS), side(MS), theta(MS)
        '''
        if self._enable_resampling:
            resampled = self._resample(values, timestamps)
        else:
            resampled = values

        # Unpack inputs
        left_position = resampled[0]
        right_position = resampled[1]
        side_position = resampled[2]
        theta = resampled[3]

        # position here is the rotoation of th wheel so needs to be multiplied by any gear ratio if present
        left_position *= self.fwd_gear_ratio
        right_position *= self.fwd_gear_ratio
        side_position *= self.side_gear_ratio

        delta_left = left_position - self.previous_left_position
        delta_right = right_position - self.previous_right_position
        delta_side = side_position - self.previous_side_position
        delta_theta = theta - self.previous_theta

        # delta_forward and delta_strafe will be the piecewise motion of this robot in this timestop, for forward and sideways/strafe in mm
        if self.fwd_is_odom:
            delta_forward = self.fwd_wheel_size * delta_left
        else:
            delta_forward = self.fwd_wheel_size * (delta_left + delta_right) / 2.0
        delta_strafe = self.side_wheel_size * delta_side

        self.x, self.y, _ = self._calc_timestep_arc_chord(self.x, self.y, self.theta, delta_forward, delta_strafe, delta_theta)
        self.theta = theta # always use gyro theta

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_side_position = side_position
        self.previous_theta = theta
    
    def _calc_timestep_arc_chord_fwd_only(self, x, y, theta, delta_forward, delta_side, delta_theta):
        '''
        ### INTERNAL

        x, y, delta_forward, delta_side in MM

        theta, delta_theta in radians
        '''

        # local deltas
        if (delta_theta == 0.0):
            # no turn - use simple deltas
            delta_local_x = delta_forward
            to_global_rotation_angle = theta
        else:
            # robot turning
            # calculate radius of movement for forward and side wheels
            r_linear = -self.fwd_offset + (delta_forward / delta_theta) # mm

            # calculate chord distances using chord length = 2 * r * sin(theta / 2)
            # pre-rotate by half the turn angle so we have only distance along one axis for each
            # when we rotate to global frame we need to account for this half-angle rotation
            to_global_rotation_angle = theta + delta_theta / 2
            delta_local_x = r_linear * 2.0 * sin(delta_theta / 2.0)

        # rotate to global
        delta_global_x = delta_local_x * cos(to_global_rotation_angle)
        delta_global_y = delta_local_x * sin(to_global_rotation_angle)

        return (x + delta_global_x, y + delta_global_y, theta + delta_theta)

    def _update_location_fwd_only(self, values, timestamps):
        '''
        ### INTERNAL

        ### Arguments
            values: array of 4 values: left(REV), right(REV), side(REV), theta(RADIANS)
            timestamps: array of 4 timestamps: left(MS), right(MS), side(MS), theta(MS)
        '''
        if self._enable_resampling:
            resampled = self._resample(values, timestamps)
        else:
            resampled = values

        # Unpack inputs
        left_position = resampled[0]
        right_position = resampled[1]
        theta = resampled[3]

        # position here is the rotoation of th wheel so needs to be multiplied by any gear ratio if present
        left_position *= self.fwd_gear_ratio
        right_position *= self.fwd_gear_ratio

        delta_left = left_position - self.previous_left_position
        delta_right = right_position - self.previous_right_position
        delta_theta = theta - self.previous_theta

        # delta_forward and delta_strafe will be the piecewise motion of this robot in this timestop, for forward and sideways/strafe in mm
        if self.fwd_is_odom:
            delta_forward = self.fwd_wheel_size * delta_left
        else:
            delta_forward = self.fwd_wheel_size * (delta_left + delta_right) / 2.0

        self.x, self.y, _ = self._calc_timestep_arc_chord_fwd_only(self.x, self.y, self.theta, delta_forward, 0.0, delta_theta)
        self.theta = theta # always use gyro theta

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_theta = theta

    def get_orientation(self):
        '''
        ### Docstring for get_orientation
        
        :returns: Tracking.Orientation Tuple
        '''
        return Tracking.Orientation(self.x, self.y, InertialWrapper.to_heading(degrees(self.theta)))

    def set_orientation(self, orientation: Orientation):
        '''
        ### Docstring for set_orientation
        
        :param self: Description
        :param orientation: Description
        :type orientation: Orientation
        '''
        self.x = orientation.x
        self.y = orientation.y
        if (orientation.heading is not None):
            self.theta = radians(InertialWrapper.to_angle(orientation.heading))
            self.previous_theta = self.theta
            self._set_sensor_heading(orientation.heading)
        self._init_rolling_buffers(
            Tracking.EncoderValues(0.0, 0.0, 0.0, Tracking.gyro_theta(self.inertial)),
            Tracking.EncoderValues(0, 0, 0, self.inertial.timestamp()), 3)

    def trajectory_to_point(self, x: float, y: float, reverse: bool = False):
        '''
        ### Returns the distance (in MM) and relative heading (in DEGREES) to the specified coordinate.

        By default the robot is assumed to be driving forward to the target point. Set the "reverse" argument to\
        True if the robot needs to reverse to the target.
        
        :param x: X or NORTH-SOUTH axis coordinate in MM
        :param y: Y or EAST-WEST axis coordinate in MM
        :param (optional) reverse: Indicates that the robot will drive in reverse

        :returns distance, heading: Tuple of distance in MM and relative heading in DEGREES to target.\
        If "reverse" is True then distance will be negated and the heading will reflect the direction the\
        front of the robot needs to be pointing meaning heading-180
        '''
        distance = sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
        heading = InertialWrapper.to_heading(degrees(atan2(y - self.y, x - self.x)))
        if reverse is True:
            distance = -distance
            heading = InertialWrapper.to_heading(heading + 180.0)

        return distance, heading

    def _rotation_local_to_global(self, theta: float):
        '''
        ### INTERNAL

        Returns rotation matrix to convert local robot coordinates to global coordinates for given angle

        Returned matrix is a 3x3 list for: [x, y, theta]\\
        Theta will be unchanged
        
        :param theta: Current angle in radians        
        '''
        R = [
            [cos(theta), -sin(theta), 0.0],
            [sin(theta), cos(theta), 0.0],
            [0.0, 0.0, 1.0]
        ]
        return R
    
    def point_on_robot(self, x: float, y: float):
        '''
        ### Given a point relative to the robot center, return the global coordinates of that point

        :param x: X coordinate relative to robot tracking center in MM
        :param y: Y coordinate relative to robot tracking center in MM
        :returns: Global X, Y coordinates in MM
        '''
        pose = [self.x, self.y, self.theta]
        offset = [x, y, 0.0]

        R = self._rotation_local_to_global(pose[2])

        # dot product
        global_offset = [
            offset[0] * R[0][0] + offset[1] * R[0][1],
            offset[0] * R[1][0] + offset[1] * R[1][1],
            0.0
        ]

        # add to current global position
        global_point = [
            pose[0] + global_offset[0],
            pose[1] + global_offset[1],
            pose[2] + global_offset[2]]
        
        return global_point[0], global_point[1]

    @staticmethod
    def gyro_theta(sensor: InertialWrapper):
        '''
        ### Docstring for gyro_theta
        
        :param sensor: Description
        :type sensor: InertialWrapper
        '''
        return radians(Tracking.gyro_rotation(sensor))

    @staticmethod
    def gyro_rotation(sensor: InertialWrapper):
        '''
        ### Docstring for gyro_rotation
        
        :param sensor: Description
        :type sensor: InertialWrapper
        '''
        return sensor.rotation()
    
    def _set_sensor_heading(self, heading):
        '''
        ### INTERNAL Docstring for set_sensor_heading
        
        :param self: Description
        :param heading: Description
        '''
        if self.inertial is None: return
        angle = InertialWrapper.to_angle(heading)
        self.inertial.set_rotation(angle, RotationUnits.DEG)

    def _avg_motor_times(self, mg: MotorGroup):
        '''
        ### INTERNAL Docstring for avg_motor_times
        
        :param self: Description
        :param mg: Description
        :type mg: MotorGroup
        '''
        num_motors = len(mg._motors)
        if num_motors == 0:
            raise RuntimeError("No Motors Found")
        sum = 0
        for m in mg._motors:
            sum += m.timestamp()
        sum = sum / len(mg._motors)
        return sum

    def _track_motors(self, left_drive: MotorGroup, right_drive: MotorGroup, inertial: InertialWrapper):
        '''
        ### INTERNAL Docstring for track_motors
        
        :param self: Description
        :param left_drive: Description
        :type left_drive: MotorGroup
        :param right_drive: Description
        :type right_drive: MotorGroup
        :param inertial: Description
        :type inertial: InertialWrapper
        '''
        print("Tracker Using Motor Encoders")

        first_run = True
        run_time_sum = 0
        run_time_count = 0
        loop_time_sum = 0
        loop_count = 0

        while(True):
            run_time_start = self.timer.system_high_res()

            if (self._is_enabled):
                values = [left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), 0.0, Tracking.gyro_theta(inertial)]
                timestamps = [self._avg_motor_times(left_drive), self._avg_motor_times(right_drive), 0, inertial.timestamp()]
                if first_run:
                    last_timestamps = [0, 0, 0, 0]
                    self._set_initial_values(Tracking.EncoderValues(values[0], values[1], values[2], values[3]),
                                             Tracking.EncoderValues(timestamps[0], timestamps[1], timestamps[2], timestamps[3]))

                updated = False
                for i in range(4):
                    if timestamps[i] != last_timestamps[i]: updated = True
                last_timestamps = timestamps

                if updated and not first_run:
                    self._update_location_fwd_only(values, timestamps)

                    run_time_end = self.timer.system_high_res()
                    run_time_sum += run_time_end - run_time_start
                    run_time_count += 1
                    self.avg_time = run_time_sum / run_time_count
                
                if updated:
                    if not first_run:
                        loop_time_sum += (run_time_start - last_run_time_start) / 1000.0
                        self.avg_rate = loop_time_sum / loop_count
                    last_run_time_start = run_time_start
                    loop_count += 1

                first_run = False
            else:
                first_run = True

            wait(self.timestep, MSEC)

    def _track_odometry(self, rotation_fwd: Rotation, rotation_side: Rotation, inertial: InertialWrapper):
        '''
        ### INTERNAL Docstring for track_odometry
        
        :param self: Description
        :param rotation_fwd: Description
        :type rotation_fwd: Rotation
        :param rotation_side: Description
        :type rotation_side: Rotation
        :param inertial: Description
        :type inertial: InertialWrapper
        '''
        print("Tracker Using Rotation Sensors")

        first_run = True
        run_time_sum = 0
        run_time_count = 0
        loop_time_sum = 0
        loop_count = 0

        while(True):
            run_time_start = self.timer.system_high_res()

            if (self._is_enabled):
                values = [rotation_fwd.position(RotationUnits.REV), 0.0, rotation_side.position(RotationUnits.REV), Tracking.gyro_theta(inertial)]
                timestamps = [rotation_fwd.timestamp(), 0, rotation_side.timestamp(), inertial.timestamp()]
                if first_run:
                    last_timestamps = [0, 0, 0, 0]
                    self._set_initial_values(Tracking.EncoderValues(values[0], values[1], values[2], values[3]),
                                             Tracking.EncoderValues(timestamps[0], timestamps[1], timestamps[2], timestamps[3]))

                updated = False
                for i in range(4):
                    if timestamps[i] != last_timestamps[i]: updated = True
                last_timestamps = timestamps

                if updated and not first_run:
                    if self.side_wheel_size == 0.0:
                        self._update_location_fwd_only(values, timestamps)
                    else:
                        self._update_location(values, timestamps)

                    run_time_end = self.timer.system_high_res()
                    run_time_sum += run_time_end - run_time_start
                    run_time_count += 1
                    self.avg_time = run_time_sum / run_time_count

                if updated:
                    if not first_run:
                        loop_time_sum += (run_time_start - last_run_time_start) / 1000.0
                        self.avg_rate = loop_time_sum / loop_count
                    last_run_time_start = run_time_start
                    loop_count += 1

                first_run = False
            else:
                first_run = True

            wait(self.timestep, MSEC)

    def _tracker_thread(self, devices, unused):
        '''
        ### INTERNAL Docstring for tracker_thread
        
        :param self: Description
        :param devices: Description
        :param unused: Description
        '''
        if len(devices) != 3:
            print("missing prequisite number of devices (3)")
            return
        
        wait(10, MSEC)

        if not self.fwd_is_odom:
            self._track_motors(devices[0], devices[1], devices[2])
        else:
            self._track_odometry(devices[0], devices[1], devices[2])

    def start_tracker(self, devices):
        '''
        ### Docstring for start_tracker
        
        :param self: Description
        :param devices: Description
        '''
        return Thread(self._tracker_thread, (devices, 0))
