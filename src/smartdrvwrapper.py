from vex import *
from motorgroup import * # pyright: ignore[reportMissingImports]
from drivetrain import * # pyright: ignore[reportMissingImports]
from smartdrive import * # pyright: ignore[reportMissingImports]
from inertialwrapper import InertialWrapper
from driveproxy import DriveProxy

class SmartDriveWrapper(SmartDrive):
    '''
    ### SmartDriveWrapper class - use this to create a smart(er) drivetrain

    This class inherits the VEX SmartDrive class that provides basic inertial sensor functionality for turning.

    #### Class Hierarchy:
        - Drivetrain (vex)
            - SmartDrive (vex)
                - SmartDriveWrapper (this class)

    This version extends the SmartDrive functionality by impplementing heading hold when driving straight through
    the drive_straight_for() function.
    
    It also implements full control over PID supporting Kp, Ki and Kd for turning, driving
    and heading hold. Three new functions are added to support this:\\
        - set_turn_constants()\\
        - set_drive_constants()\\
        - set_heading_lock_constants()\\

    It is also based on the InertialWrapper class that provides scaling for gyros with a (constant) heading error.

    The GPS sensor is currently not suppoorted.

    :param lm: Left motorgroup
    :type lm: MotorGroup
    :param rm: Right motorgroup
    :type rm: MotorGroup
    :param g: Inertial sensor
    :type g: InertialWrapper
    :param (optional) wheelTravel:  The circumference of the driven wheels, default is 320 mm (~4" * pi)
    :param (optional) trackWidth: The trackwidth of the drivetrain, default is 254 mm (10")
    :param (optional) wheelBase:  The wheelBase of the drivetrain, default is 254 mm (10)
    :param (optional) units: The units that wheelTravel, trackWidth and wheelBase are specified in, only MM is supported
    :param (optional) externalGearRatio: Gear ratio used to compensate drive distances if gearing is used
    :returns out: A new SmartDriveWrapper object
    '''

    class TurnMode:
        '''
        ### TurnMode Enum for overriding default turn control mode on a turn by turn basis

        VOLTAGE (default) will command motors using VOLTAGE. 
        PERCENT will command motors using PERCENT.
        SMART will use the base SmartDrive.turn_for() method (current not implemented)
        '''
        VOLTAGE = 0, # uses voltage control
        PERCENT = 1, # uses percent control

    def __init__(self,
                 lm: MotorGroup,
                 rm: MotorGroup,
                 g: InertialWrapper,
                 wheelTravel = 320.0,
                 trackWidth = 254.0,
                 wheelBase = 254.0,
                 units = DistanceUnits.MM,
                 externalGearRatio = 1.0):
   
        if units != DistanceUnits.MM:
            raise ValueError("SmartDriveWrapper only supports MM for units")
        if not isinstance(g, InertialWrapper):
            raise TypeError("g must be an InertialWrapper instance")
        if not isinstance(lm, MotorGroup) or not isinstance(rm, MotorGroup):
            raise TypeError("Only supports MotorGroup instances for lm and rm")
        
        self._turn_velocity = 66 # PERCENT
        self._drive_velocity = 66 # PERCENT
        
        self.g = g
        self.dp = DriveProxy(lm, rm, g, wheel_travel_mm= wheelTravel, ext_gear_ratio= externalGearRatio)
        self.dp.set_turn_constants(Kp=1.0, Ki=0.04, Kd=10.0, settle_error=0.5) # degrees
        self.dp.set_drive_constants(Kp=0.5, Ki=0.0, Kd=0.0, settle_error=5) # mm
        self.dp.set_heading_lock_constants(Kp=2.0, Ki=0.0, Kd=0.0) # degrees
        self.dp.set_turn_velocity(self._drive_velocity, PERCENT)
        self.dp.set_drive_velocity(self._drive_velocity, PERCENT)
        self.dp.set_drive_acceleration(10, PERCENT) # 5% per timestep
        self.dp.set_timeout(10) # seconds
        self.dp.set_stopping(BrakeType.COAST)

        super().__init__(lm, rm, g, wheelTravel, trackWidth, wheelBase, units, externalGearRatio)
        super().set_turn_velocity(self._drive_velocity, PERCENT)
        super().set_drive_velocity(self._drive_velocity, PERCENT)
        super().set_timeout(10, SECONDS)
        super().set_turn_constant(1.0)
        super().set_turn_threshold(0.5)
        super().set_stopping(BrakeType.COAST)

    def set_turn_threshold(self, value):
        '''
        ### Set the turning threshold for the robot

        This is the threshold value used to determine that turns are complete.\\
        If this is too large then turns will not be accurate, if too small then turns may not complete.

        :param value: The new turn threshold in degrees, the default for SmartDriveWrapper is +/-1 degree
        '''
        self.dp.set_turn_constants(settle_error=value)

        super().set_turn_threshold(value)

    def set_turn_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller for turning. This requires three gain constants, generally known
        as Kp Ki and Kd. They set the gain used in the equation that turns angular error into motor velocity.

        Each robot will have a unique set of values that is mostly affected by the surface type, nmotor/gear/wheel
        configuration, robot size and weight.

        Even then these are strictly not constants as different speeds may require a different set of values.

        The settle error, or threshold, is set separately using the set_turn_threshold() method

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
        '''
        self.dp.set_turn_constants(Kp=Kp, Ki=Ki, Kd=Kd)

        super().set_turn_constant(Kp)

    def set_turn_constant(self, Kp):
        '''
        ### DEPRECATED: Use set_turn_constants()
        '''
        self.set_turn_constants(Kp=Kp)
        super().set_turn_constant(Kp)

    def set_drive_threshold(self, value):
        '''
        ### Set the drive threshold for the robot

        This is the distance threshold value used to determine that drives are complete.\\
        If this is too large then drive distance will not be accurate, if too small then drives may not complete.

        Note that internally within the motor the best accuracy is limited by the encoder which has a resolution
        of 3.6 degrees, or 0.2deg (+/-0.1deg) for a green catridge and 0.6deg (+/-0.3deg) for a blue cartdige.\\
        The impact of this will depend on the wheel size and the external gearing. Some examples are given below for ideal
        conditions. However backlash (gear slop) will add 50x-100x more uncertainty than this making a practical limit for the
        threshold at around +/-5mm.

        #### Examples for maximum resolution:
            - 4" wheels, 18:1 internal gearing, no external gearing: +/- 0.089mm (= +/-0.1deg at motor)
            - 3.25" wheels, 6:1 internal gearing, 24:60 external gearing: +/- 0.087mm (= +/-0.3deg)

         :param value: The new drive threshold in MM, the default is +/-5mm
        '''
        self.dp.set_drive_constants(settle_error=value)

    def set_drive_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the driving constants for the robot

        SmartdriWewrapper uses a PID controller when driving..\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns distance error into motor velocity.

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
        '''
        self.dp.set_drive_constants(Kp=Kp, Ki=Ki, Kd=Kd)

    def set_heading_lock_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the constants used for driving straight, or heading lock/hold

        SmartdriWewrapper uses PID controllers for turns and driving.\\
        When we want the robot to drive straight we use both controllers at the same time.\\
        The main difference between individual turns and drives is that the heading lock does not have a
        timeout or a settle error (threshold) associated with it.

        Kp is often only used and is set more aggressively than for just individual turns.

        This call replaces the turn constants only for driving straight (drive constants are unaffected)

        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
        '''

        self.dp.set_heading_lock_constants(Kp=Kp, Ki=Ki, Kd=Kd)

    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### Set the inertial sensor heading to a new value

        The new value for heading should be in the range [0, 360) degrees.

        :param value: The new value to use for heading.
        :param (optional) units: The rotation units type for value, only DEGREES is supported
        '''

        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper only supports DEGREES for heading")

        super().set_heading(value, units)
    
    def heading(self, units=RotationUnits.DEG):
        '''
        ### Read the current heading of the inertial sensor

        Heading will be returned in the range [0, 360) degrees

        :param (optional) units: The units to return the heading in, only DEGREES is supported
        :returns heading: in DEGREES
        '''
        return super().heading(units)
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### Set the inertial sensor rotation to a new value

        :param value: The new value to use for rotation.
        :param (optional) units: The rotation units type for value, only DEGREES is supported
        '''
        super().set_rotation(value, units)

    def rotation(self, units=RotationUnits.DEG):
        '''
        ### Read the current rotation of the inertial sensor

        rotation is unbounded, meaning it can be in the range [-inf, inf]. This is generally more useful for performing
        calculations on a heading as there are no discontinuities when going from (say) a heading of 359 to 0 degrees.

        heading is just the same value as rotation but "reduced" and offset so that it is in the range of [0, 359). heading
        is more useful for specifying directions for the robot

        :param (optional) units: The rotation units type for value, only DEGREES is supported
        :returns: A value for heading in the range that is specified by the units.
        '''
        return super().rotation(units)
    
    def set_turn_direction_reverse(self, value):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("set_turn_direction_reverse() not supported")

    def turn_to_heading(self, heading, units=RotationUnits.DEG,
                        velocity=None, units_v:VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                        wait=True, mode=TurnMode.VOLTAGE):
        '''
        ### turn the robot to an absolute heading

        The angle and direction by which to turn the robot is calculated automatically. It will turn by the smallest
        angle to get to the specified heading. A difference of exactly 180deg will turn to the RIGHT.

        :param heading: The heading to turn to in the range [0, 360)
        :pafram (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
        :param (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :param (optional) mode: TurnMode to use VOLTAGE, PERCENT or SMART (default is VOLTAGE)
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
        '''

        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT and units_v != VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_to_heading(heading, settle_error=None, timeout=None, wait=wait, use_voltage=(mode==self.TurnMode.VOLTAGE))
    
    def turn_to_rotation(self, rotation, units=RotationUnits.DEG,
                         velocity=None, units_v:  VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                         wait=True, mode=TurnMode.VOLTAGE):
        '''
        ### turn the robot to an absolute rotation

        Care should be taken when using this version as rotation can be outside of the range -360,360 deg if the robot has
        performed one or more complete turns. E.g. if the current rotation is 360deg and 0 deg is specified, the robot will
        turn by 360deg (not zero) to make the rotation match the target

        :param rotation: The rotation to turn to
        :pafram (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
        :param (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :param (optional) mode: TurnMode to use VOLTAGE, PERCENT or SMART (default is VOLTAGE)
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
        '''

        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT and units_v != VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_to_rotation(rotation, settle_error=None, timeout=None, wait=wait, use_voltage=(mode==self.TurnMode.VOLTAGE))

    def turn_for(self, direction, angle, units = RotationUnits.DEG,
                 velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                 wait=True, mode=TurnMode.VOLTAGE):
        '''
        ### Turn the robot for an angle left or right

        The robot is commanded to turn by specified angle either LEFT or RIGHT. Note that turning by negative amount\\
        is the same as providing a positive value but switching direction from LEFT to RIGHT or vice-versa

        :param direction: The direction to turn, LEFT or RIGHT
        :param angle: The angle to turn
        :param (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
        :parm (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :param (optional) mode: TurnMode to use VOLTAGE, PERCENT or SMART (default is VOLTAGE)
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
        '''

        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT and units_v != VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_for(direction, angle, units, settle_error=None, timeout=None, wait=wait, use_voltage=(mode==self.TurnMode.VOLTAGE))

    def turn(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("turn() not supported")

    def drive_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  wait=True):
        '''
        ### Drives the robot for the distance specified

        If velocity is not specified the value provided by set_drive_velocity() will be used.

        Internally the function uses the wheelSize and externalGearRatio specified upon object creation to determine how
        many revolutions the motor needs to turn for, so make sure these are set correctly
        
        :param direction: FOWARD or REVERSE
        :param distance: Distnace in MM the robot should drive. Providing negative distance will drive in the opposite direction specified by the direction parameter
        :param (optional) units: Only MM supported
        :param velocity: Description
        :param units_v: Description
        :type units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits
        :param wait: Description
        :return: Description
        :rtype: Any | Literal[True]
        '''
        if velocity is not None: raise NotImplementedError("SmartDriveWrapper.drive_for(): velocity parameter not supported, use set_drive_velocity() instead")
        return self.dp.drive_for(direction, distance, units, heading=None, settle_error=None, timeout=None, wait=wait)
        #return super().drive_for(direction, distance, units, velocity, units_v, wait)

    def drive_straight_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  heading = None, units_h = RotationUnits.DEG,
                  wait=True):
        '''
        ### Docstring for drive_straight_for
        
        :param direction: Description
        :param distance: Description
        :param units: Description
        :param velocity: Description
        :param units_v: Description
        :type units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits
        :param heading: Description
        :param units_h: Description
        :param wait: Description
        :return: Description
        :rtype: Any | Literal[True]
        '''
        if velocity is not None: raise NotImplementedError("SmartDriveWrapper.drive_for(): velocity parameter not supported, use set_drive_velocity() instead")
        if velocity is not None and units_v != VelocityUnits.PERCENT and units_v != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.drive_straight_for(): Only PERCENT supported for units_v")
        if heading is not None and units_h != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.drive_straight_for(): Only DEGREES supported for units_h")
        return self.dp.drive_for(direction, distance, units, heading=heading, settle_error=None, timeout=None, wait=wait)

    def drive_to_point(self, x: float, y: float, direction, orientation_callback: Callable, turn_limit: float = 0.0, wait = True):
        '''
        ### Drive robot to the x/y point in MM specified

        A callback function must be provided that will return the current x,y and heading of the robot when called.\\
        This function can drive forward or backward to the target point as specified by the direction parameter.\\
        The function will return when the robot crosses a line perpendicular to the path to the target point or when the distance
        error is within the settle error specified by set_drive_threshold(). Typically the line crossing will occur first so that
        the robot position may not be quite as accurate as performing a turn followed by a drive. However this method is generally
        much faster when chaining multiple drive_to_point commands together where intermeddiate accuracy is not so important.
        
        :param x: X (NORTH) coordinate of target point in MM
        :type x: float
        :param y: Y (EAST) coordinate of target point in MM
        :type y: float
        :param direction: FOWARD or REVERSE
        :param orientation_callback: Callback function that returns current x,y,heading (in DEGREES) of the robot
        :type orientation_callback: Callable
        :param turn_limit: Dampens the turning response as robot gets closer to target to prevent overcorrections in heading. MM from target at which turn response trails off
        :param wait: When True (default) the function will wait for the command to complete before returning
        :return: Time for command to complete in MS (if wait=True)
        :rtype: Any | Literal[False]
        '''
        return self.dp.drive_to_point(x, y, direction, orientation_callback, settle_error = None, turn_limit = turn_limit, timeout = None, wait = wait)

    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("drive() not supported")

    def is_turning(self):
        '''
        ### DEPRECATED: Use is_done()
        '''
        raise NotImplementedError("is_turning() not supported")
        # return self.is_done()

    def is_moving(self):
        '''
        ### DEPRECATED: Use id_done() 
        '''
        raise NotImplementedError("is_moving() not supported")
        # return self.is_done()

    def is_done(self):
        '''
        ### Returns the current status of the drive_for or turn_for command

        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return False if the drivetrain is still moving or True if it has completed the move or a timeout occurred.

        :returns: The current drive_for or turn_for status
        '''
        return self.dp.is_done()

    def set_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for drive commands

        This will be the velocity used for subsequent calls to drive if a velocity is not provided
        to that function.

        :param velocity: The new velocity
        :param (optional) units: Only PERCENT is supported
        '''
        if units != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_drive_velocity(): PERCENT supported for units")

        self._drive_velocity = velocity
        self.dp.set_drive_velocity(self._drive_velocity, units)
        super().set_drive_velocity(self._drive_velocity, units)

    def set_min_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set the minimum velocity for drive commands

        This will be the minimum velocity used for subsequent calls to drive commands that use minimum velocity for fast chaining.
        Currently only drive_to_point() support this.

        :param velocity: The new minimum velocity
        :param (optional) units: Only PERCENT is supported
        '''
        if units != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_min_drive_velocity(): PERCENT supported for units")

        self.dp.set_min_drive_velocity(velocity, units)

    def set_drive_acceleration(self, accel, units:VelocityPercentUnits = VelocityUnits.PERCENT):
        '''
        ### Set default acceleration for drive commands

        This will be the accelertaion used for subsequent calls to drive

        :param accel: The new acceleration in PERCENT per timestep
        :param (optional) units: Only PERCENT is supported
        '''
        if units is not PercentUnits.PERCENT and units is not VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_drive_velocity(): PERCENT supported for units")

        self.dp.set_drive_acceleration(accel, units)

    def set_turn_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for turn commands

        This will be the velocity used for subsequent calls to turn if a velocity is not provided
        to that function.

        Note that turn_velocity affects both pure turns (e.g. turn_for()) and turns
        that occur during drive commands when heading hold is used (drive_straight_for() and drive_to_point())

        :param velocity: The new velocity
        :param (optional) units: The units for the supplied velocity, (PERCENT only))
        '''
        if units != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_turn_velocity(): PERCENT supported for units")
        
        self._turn_velocity = velocity
        self.dp.set_turn_velocity(self._turn_velocity, units)
        super().set_turn_velocity(self._turn_velocity, units)

    def set_stopping(self, mode=BrakeType.COAST):
        '''
        ### Set the stopping mode for all motors on the drivetrain

        Setting the action for the motors when stopped.

        :param (optional) mode: The stopping mode, COAST (default), BRAKE or HOLD
        '''
        self.dp.set_stopping(mode)
        super().set_stopping(mode)

    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''
        ### Set the timeout value used all motors on the drivetrain
        
        The timeout value is used when performing drive_for and turn_for commands.  If timeout is
        reached and the motor has not completed moving, then the function will return False.

        timeout : The new timeout
        units : The units for the provided timeout, the default is MSEC
        '''
        super().set_timeout(timeout, units)
        if units == TimeUnits.MSEC: timeout = timeout / 1000.0
        self.dp.set_timeout(timeout)

    def get_timeout(self):
        '''
        ### Get the current timeout value used by the drivetrain

        :returns: Timeout value in mS
        '''
        return self.dp.get_timeout()

    def on_timeout(self, fn):
        '''
        ### INTERNAL
        
        :param self: Description
        :param fn: Description
        '''
        self.dp.set_timeout_callback(fn)

    def stop(self, mode=None):
        '''
        ### Stop the drivetrain
        
        This will override any running drive or turn commands (as long as they are interruptable, wait=True)

        Velocity is set to 0 and the specified mode (if present) is used to stop the motors.
        
        If mode is not specified, then the default action that was set using set_stopping() will be used.

        :param (optional) mode: The stopping mode, COAST, BRAKE or HOLD
        '''
        self.dp.stop(mode)

# ----------------------------------------------------------------------------

    # def velocity(self, units:VelocityPercentUnits=VelocityUnits.RPM):
    # def current(self, units=CurrentUnits.AMP)
    # def power(self, units=PowerUnits.WATT)
    # def torque(self, units=TorqueUnits.NM)
    # def efficiency(self, units=PercentUnits.PERCENT)
    # def temperature(self, units=TemperatureUnits.CELSIUS)
