from vex import *
from motorgroup import * # pyright: ignore[reportMissingImports]
from drivetrain import * # pyright: ignore[reportMissingImports]
from smartdrive import * # pyright: ignore[reportMissingImports]
from inertialwrapper import InertialWrapper
from  driveproxy import DriveProxy

class SmartDriveWrapper(SmartDrive):
    '''
    ### SmartDriveWrapper class - use this to create a smart(er) drivetrain

    The smart drivetrains use an inertial sensor for heading, taking the guesswork
    out of turning the robot. This version extends the base SmartDrive functionality
    by impplementing heading hold when driving straight and full control over PID.

    It is also based on the InertialWrapper class that provides scaling for gyros with
    a (constant) heading error.

    #### Arguments:
    - lm : Left motorgroup
    - rm : Right motorgroup
    - g : Inertial sensor
    - wheelTravel (optional) : The circumference of the driven wheels, default is 320 mm (~4" * pi)
    - trackWidth (optional) : The trackwidth of the drivetrain, default is 254 mm (10")
    - wheelBase (optional) : The wheelBase of the drivetrain, default is 254 mm (10)
    - units (optional) : The units that wheelTravel, trackWidth and wheelBase are specified in, only MM is supported
    - externalGearRatio (optional) : An optional gear ratio used to compensate drive distances if gearing is used

    #### Returns:
        A new SmartDriveWrapper object.
    '''
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

        super().__init__(lm, rm, g, wheelTravel, trackWidth, wheelBase, units, externalGearRatio)
        super().set_turn_velocity(self._drive_velocity, PERCENT)
        super().set_drive_velocity(self._drive_velocity, PERCENT)

    def set_turn_threshold(self, value):
        '''
        ### Set the turning threshold for the robot

        This is the threshold value used to determine that turns are complete.\\
        If this is too large then turns will not be accurate, if too small then turns ma\\
        not complete.

        #### Arguments:
            value : The new turn threshold in degrees, the default for SmartDriveWrapper is +/-1 degree

        #### Returns:
            None
        '''
        self.dp.set_turn_constants(settle_error=value)

        super().set_turn_threshold(value)

    def set_turn_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller when doing turns.\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        The settle error or threshold is set separately using the set_turn_threshold() method

        #### Arguments:
            Kp : The new turn P constant
            Ki (optional) : The new turn I constant, default is 0.0
            Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        self.dp.set_turn_constants(Kp=Kp, Ki=Ki, Kd=Kd)

        super().set_turn_constant(Kp)

    def set_turn_constant(self, Kp):
        '''
        ### DEPRECATED: Use set_turn_constants()
        '''
        self.set_turn_constants(Kp=Kp)

    def set_drive_threshold(self, value):
        '''
        ### Set the drive threshold for the robot

        This is the distance threshold value used to determine that drives are complete.\\
        If this is too large then drive distance will not be accurate, if too small then drives may not complete.

        Note that internally within the motor the best accuracy is limited by the encoder which has a resolution
        of 3.6 degrees, or 0.2deg (+/-0.1deg) for a green catridge and 0.6deg (+/-0.3deg) for a blue cartdige.\\
        The impact of this will depend on the wheel size and the external gearing. Some examples are given below for ideal
        conditions. However backlash (gear slop) will add much more uncertainty than this making a practical limit for the
        threshold at around +/-5mm.

        #### Arguments:
        - value : The new drive threshold in MM, the default is +/-5mm

        #### Returns:
            None

        #### Examples for maximum resolution:
            - 4" wheels, 18:1 internal gearing, no external gearing: +/- 0.089mm (= +/-0.1deg at motor)
            - 3.25" wheels, 6:1 internal gearing, 24:60 external gearing: +/- 0.087mm (= +/-0.3deg)
        '''
        self.dp.set_drive_constants(settle_error=value)

    def set_drive_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller when driving.\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns distance error into motor velocity.

        #### Arguments:
            Kp : The new turn P constant
            Ki (optional) : The new turn I constant, default is 0.0
            Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        self.dp.set_drive_constants(Kp=Kp, Ki=Ki, Kd=Kd)

    def set_headling_lock_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the constants used for driving straight, or heading lock/hold

        SmartdriWewrapper uses PID controllers for turns and driving.\\
        When we want the robot to drive straight we use both controllers at the same time.\\
        The main difference between
        individual turns and drives is that the heading lock does not have a timeout or a settle error (threshold)
        associated with it.
        Kp is often only used and is set more aggressively than for just individual turns.

        This call replaces the turn constants only for driving straight (drive constants are unaffected)

        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        #### Arguments:
        - Kp : The new turn P constant
        - Ki (optional) : The new turn I constant, default is 0.0
        - Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        self.dp.set_heading_lock_constants(Kp=Kp, Ki=Ki, Kd=Kd)

    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### set the inertial sensor heading to a new value

        The new value for heading should be in the range [0, 360) degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            smart1.set_heading(180)
        '''

        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper only supports DEGREES for heading")

        super().set_heading(value, units)
    
    def heading(self, units=RotationUnits.DEG):
        '''
        ### read the current heading of the inertial sensor

        heading will be returned in the range [0, 360) degrees

        #### Arguments:
            units (optional) : The units to return the heading in, only DEGREES is supported

        #### Returns:
            heading in DEGREES

        #### Examples:
            # get the current heading for the robot\\
            value = smart1.heading()
        '''
        return super().heading(units)
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### set the inertial sensor rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            smart1.set_rotation(180)
        '''
        super().set_rotation(value, units)

    def rotation(self, units=RotationUnits.DEG):
        '''
        ### read the current rotation of the inertial sensor

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gyro.

        #### Arguments:
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the robot\\
            value = smart1.rotation()
        '''
        return super().rotation(units)
    
    def set_turn_direction_reverse(self, value):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("set_turn_direction_reverse() not supported")

    def turn_to_heading(self, heading, units=RotationUnits.DEG,
                        velocity=None, units_v:VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### turn the robot to an absolute position

        This function uses the value of heading() when turning the robot

        #### Arguments:
            heading : The heading to turn to
            units (optional) : The units for the provided angle (DEGREES only)
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocit (PERCENT only)
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None
        '''
        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_to_heading(heading, settle_error=None, timeout=None, wait=wait)
        # return super().turn_to_heading(heading, units, velocity, units_v, wait)

    def turn_to_rotation(self, rotation, units=RotationUnits.DEG,
                         velocity=None, units_v:  VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### turn the robot to an absolute rotation

        This function uses the value of rotation() when turning the robot

        #### Arguments:
            rotation : The angle to turn to
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
        '''
        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_to_rotation(rotation, settle_error=None, timeout=None, wait=wait)

    def turn_for(self, direction, angle, units = RotationUnits.DEG,
                 velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### Turn the robot for an angle left or right

        The robot is commanded to turn by specified angle either LEFT or RIGHT. Note that turning by negative amount\\
        is the same as providing a positive value but switching direction from LEFT to RIGHT or vice-versa

        #### Arguments:
        1. direction : The direction to turn, LEFT or RIGHT
        2. angle : The angle to turn
        3. units (optional) : The units for the provided angle (DEGREES only)
        4. velocity (optional) : drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
        5. units_v (optional) : The units of the provided velocity (PERCENT only)
        6. wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None or if wait is True then completion success or failure

        #### Examples:
        '''
        if units != RotationUnits.DEG:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only DEGREES supported for units")
        if units_v != PercentUnits.PERCENT or units_v != VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.turn_to_heading(): Only PERCENT supported for units_v")

        if velocity is not None:
            self.dp.set_turn_velocity(velocity, units_v)
        else:
            self.dp.set_turn_velocity(self._turn_velocity, PercentUnits.PERCENT)
        return self.dp.turn_for(direction, angle, units, settle_error=None, timeout=None, wait=wait)
    
        # return super().turn_for(direction, angle, units, velocity, units_v, wait)

    def turn(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("turn() not supported")

    def drive_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  wait=True):
        self.dp.drive_for(direction, distance, units, heading=None, settle_error=None, timeout=None, wait=wait)
        #return super().drive_for(direction, distance, units, velocity, units_v, wait)

    def drive_straight_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  heading = None, units_h = RotationUnits.DEG,
                  wait=True):
        return super().drive_for(direction, distance, units, velocity, units_v, wait)

    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        raise NotImplementedError("drive() not supported")

    def is_turning(self):
        '''
        ### DEPRECATED: Use is_done()
    
        ### Returns the current status of the turn_to_heading, turn_to_rotation or turn_for command
        This function is used when False has been passed as the wait parameter to turn_to_heading or turn_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current turn_to_heading, turn_to_rotation or turn_for status
        '''
        return self.is_done()

    def is_moving(self):
        '''
        ### DEPRECATED: Use id_done() 

        ### Returns the current status of the drive_for command
        This function is used when False has been passed as the wait parameter to drive_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for status
        '''
        return self.is_done()

    def is_done(self):
        '''
        ### Returns the current status of the drive_for or turn_for command
        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return False if the drivetrain is still moving or True if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for or turn_for status
        '''
        return self.dp.is_done()

    def set_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for drive commands
        This will be the velocity used for subsequent calls to drive if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : Only PERCENT is supported

        #### Returns:
            None
        '''
        if units != PercentUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_drive_velocity(): PERCENT supported for units")

        self._drive_velocity = velocity
        self.dp.set_drive_velocity(self._drive_velocity, units)
        super().set_drive_velocity(self._drive_velocity, units)

    def set_drive_accleration(self, accel, units:VelocityPercentUnits = VelocityUnits.PERCENT):
        '''
        ### Set default acceleration for drive commands

        This will be the accelertaion used for subsequent calls to drive

        #### Arguments:
            accel : The new acceleration in PERCENT per timestep
            units : Only PERCENT is supported

        #### Returns:
            None
        '''
        if units is not PercentUnits.PERCENT and units is not VelocityUnits.PERCENT:
            raise ValueError("SmartDriveWrapper.set_drive_velocity(): PERCENT supported for units")

        self.dp.set_drive_acceleration(accel, units)

    def set_turn_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for turn commands
        This will be the velocity used for subsequent calls to turn if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : The units for the supplied velocity, (PERCENT only))

        #### Returns:
            None
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

        #### Arguments:
            mode : The stopping mode, COAST, BRAKE or HOLD

        #### Returns:
            None
        '''
        self.dp.set_stopping(mode)
        super().set_stopping(mode)

    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''
        ### Set the timeout value used all motors on the drivetrain
        The timeout value is used when performing drive_for and turn_for commands.  If timeout is
         reached and the motor has not completed moving, then the function will return False.

        #### Arguments:
            timeout : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        super().set_timeout(timeout, units)
        if units == TimeUnits.MSEC: timeout = timeout / 1000.0
        self.dp.set_timeout(timeout)

    def get_timeout(self):
        '''
        ### Get the current timeout value used by the drivetrain

        #### Arguments:
            None

        #### Returns:
            Timeout value in mS
        '''
        return self.dp.get_timeout()

    def on_timeout(self, fn):
        self.dp.set_timeout_callback(fn)

    def stop(self, mode=None):
        '''
        ### Stop the drivetrain, set to 0 velocity and set current stopping_mode
        The motors will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        self.dp.stop(mode)

# ----------------------------------------------------------------------------

    # def velocity(self, units:VelocityPercentUnits=VelocityUnits.RPM):
    # def current(self, units=CurrentUnits.AMP)
    # def power(self, units=PowerUnits.WATT)
    # def torque(self, units=TorqueUnits.NM)
    # def efficiency(self, units=PercentUnits.PERCENT)
    # def temperature(self, units=TemperatureUnits.CELSIUS)
