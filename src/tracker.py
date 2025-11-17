# Library imports
from vex import *
from math import sin, cos, radians, degrees, atan2, sqrt
from collections import namedtuple
from inertialwrapper import InertialWrapper

# Tracking class to calculate robot position using tracking wheels or motor encoders plus inertial sensor
# Internally everything is based in meters and radians
# To distinguish from VEX uses for various heading readings, we use the following terminology:
# - ROTATION: continuous rotation value in degrees (can be positive or negative, no bounds)
# - HEADING: bounded heading in degrees [0, 360)
# - ANGLE: bounded angle in degrees (-180, +180]
# - THETA: continuous rotation value in radians (can be positive or negative, no bounds). Same as ROTATION but in radians
# THETA is used internally and converted to/from HEADING/ANGLE as needed
# Note that  __init__() and update_location() assume that the gyro scale factor has already been applied to the inertial sensor readings
class Tracking:
    
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

    Configuration = namedtuple('Configuration', [
            'fwd_is_odom',
            'fwd_wheel_size', 'fwd_gear_ratio', "fwd_offset",
            'side_wheel_size', 'side_gear_ratio', 'side_offset'
       ])
    
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

    # this_instance will hold the singleton instance of the tracker. This is a way to have all the tracking code and the associated thread
    # as part of the same class to keep all the code together
    # The tracker_thread() is started from  pre_autonomous(), or somewhere early in the program and sets this_instance once initialized
    _THIS_INSTANCE = None
    _INITIALIZED = False

    # Initializer
    # @param x is initial NORTH position in MM
    # @param y is initial EAST position in MM
    # @param heading is initial true heading of robot in degrees [0, 360). 0 deg is NORTH
    # @param configuration (optional) is the configuration of the wheels used for odometry
    # @param inital_values (optional) is the initial values of the encoders used if not zero. Not that theta (heading) will be ignored at the moment
    def __new__(cls, *args, **kwargs):
        print('new')
        if cls._THIS_INSTANCE is None:
            cls._THIS_INSTANCE = super().__new__(cls)
        return cls._THIS_INSTANCE

    def __init__(self,
                 orientation: Union[Orientation, None] = None,
                 configuration: Union[Configuration, None] = None,
                 initial_values: Union[EncoderValues, None] = None,
                 inertial: Union[InertialWrapper, None] = None):
        print('init')

        if (self._INITIALIZED): return
        self._INITIALIZED = True

        self._is_enabled = False

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
        self.inertial = inertial
        if self.inertial is None:
            raise Exception("ERROR: INERTIAL SENSOR MUST BE PRESENT ON FIRST INITIALIZATION")
        self.set_sensor_heading(heading)

        # Configuration
        self.fwd_is_odom = False
        self.fwd_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.fwd_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.fwd_offset = Tracking.DEFAULT_FWD_OFFSET

        self.side_wheel_size = Tracking.DEFAULT_WHEEL_SIZE
        self.side_gear_ratio = Tracking.DEFAULT_GEAR_RATIO
        self.side_offset = Tracking.DEFAULT_SIDE_OFFSET
        if (configuration is not None): self.set_configuration(configuration)

        self.timestep = 0.01 # seconds

        # Capture initial values of encoders and store as the previous values
        self.previous_left_position = 0.0 if initial_values is None else initial_values.left # revolutions
        self.previous_right_position = 0.0 if initial_values is None else initial_values.right # revolutions
        self.previous_side_position =  0.0 if initial_values is None else initial_values.side # revolutions
        self.previous_theta = self.theta # radians

    def set_configuration(self, configuration: Configuration):
        self.fwd_is_odom = configuration.fwd_is_odom
        self.fwd_wheel_size = configuration.fwd_wheel_size
        self.fwd_gear_ratio = configuration.fwd_gear_ratio
        self.fwd_offset = configuration.fwd_offset

        self.side_wheel_size = configuration.side_wheel_size
        self.side_gear_ratio = configuration.side_gear_ratio
        self.side_offset = configuration.side_offset
   
    def enable(self, enabled = True):
        self._is_enabled = enabled
        
    # returns internal theta (radians) in degrees heading [0, 360)
    # theoretically this is same as calling GyroHelper.gyro_heading()
    def current_heading(self):
        heading_deg = degrees(self.theta)
        return InertialWrapper.to_heading(heading_deg)

    def calc_timestep_arc_chord(self, x, y, theta, delta_forward, delta_side, delta_theta):
        # x, y, delta_forward, delta_side in MM
        # theta, delta_theta in radians

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

    def update_location(self, left_position, right_position, side_position, theta):
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

        self.x, self.y, self.theta = self.calc_timestep_arc_chord(self.x, self.y, self.theta, delta_forward, delta_strafe, delta_theta)

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_side_position = side_position
        self.previous_theta = theta
    
    def get_orientation(self):
        return Tracking.Orientation(self.x, self.y, InertialWrapper.to_heading(degrees(self.theta)))

    def set_orientation(self, orientation: Orientation):
        self.x = orientation.x
        self.y = orientation.y
        self.theta = radians(InertialWrapper.to_angle(orientation.heading))
        self.previous_theta = self.theta
        self.set_sensor_heading(orientation.heading)

    def trajectory_to_point(self, x, y):
        distance = sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
        heading = InertialWrapper.to_heading(degrees(atan2(y - self.y, x - self.x)))
        return distance, heading

    @staticmethod
    def gyro_theta(sensor: InertialWrapper):
        return radians(Tracking.gyro_rotation(sensor))

    @staticmethod
    def gyro_rotation(sensor: InertialWrapper):
        return sensor.rotation()
    
    def set_sensor_heading(self, heading):
        if self.inertial is None: return
        angle = InertialWrapper.to_angle(heading)
        self.inertial.set_rotation(angle, RotationUnits.DEG)

    @staticmethod
    def track_motors(left_drive: MotorGroup, right_drive: MotorGroup, inertial: InertialWrapper, configuration: Configuration, orientation: Orientation):
        print("Tracker Using Motor Encoders")
        initial_encoders = Tracking.EncoderValues(left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), 0.0, Tracking.gyro_theta(inertial)) 
        tracker = Tracking(orientation, configuration, initial_encoders, inertial)
        while(True):
            if (tracker._is_enabled):
                tracker.update_location(left_drive.position(RotationUnits.REV), right_drive.position(RotationUnits.REV), 0.0, Tracking.gyro_theta(inertial))
            wait(tracker.timestep, SECONDS)

    @staticmethod
    def track_odometry(rotation_fwd: Rotation, rotation_side: Rotation, inertial: InertialWrapper, configuration: Configuration, orientation: Orientation):
        print("Tracker Using Rotation Sensors")
        initial_encoders = Tracking.EncoderValues(rotation_fwd.position(RotationUnits.REV), 0.0, rotation_side.position(RotationUnits.REV), Tracking.gyro_theta(inertial)) 
        tracker = Tracking(orientation, configuration, initial_encoders, inertial)
        while(True):
            if (tracker._is_enabled):
                tracker.update_location(rotation_fwd.position(RotationUnits.REV), 0.0, rotation_side.position(RotationUnits.REV), Tracking.gyro_theta(inertial))
            wait(tracker.timestep, SECONDS)

    @staticmethod
    def tracker_thread(configuration: Configuration, devices, orientation: Orientation):
        if len(devices) != 3:
            print("missing prequisite number of devices (3)")
            return

        if not configuration.fwd_is_odom:
            Tracking.track_motors(devices[0], devices[1], devices[2], configuration, orientation)
        else:
            Tracking.track_odometry(devices[0], devices[1], devices[2], configuration, orientation)


