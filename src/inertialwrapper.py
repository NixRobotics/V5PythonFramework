from vex import Inertial, RotationUnits

class InertialWrapper(Inertial):
    '''
    ### Wraps the Inertial class to add scaling correction

    API matches that of Inertial with a couple of helper functions added.

    Upon creation a scale factor must be provided along with the port number. This is the correction that needs to \\
    be aplied to get the correct reading. E.g. if a robot it turned 360 degrees and the gyro reads 355 degrees, then \\
    the scale factor should be 360 / 355 (or 1.0141). Another way to determine this is to use a rroutine such as \\
    SmartDrive.turn_for() to turn the robot by 360 degrees. The resulting error in heading will be the inverse of \\
    the scale factor, meaning that if the robot turns by 365 degrees when instructed to turn 360 degrees, the scale \\
    factor is 365 / 360 (or 1.0139). In practice we would use several turns, say 10, in both directions as the error \\
    becomes more visible and easy to measure.

    ### Arguments
        port: port number
        gyro_scale: this is the READOUT scale factor needed to produce the correct reading for a given robot turn

    ### Returns
        Instance of InertialWrapper
    '''
    def __init__(self, port, gyro_scale):
        super().__init__(port)
        self.gyro_scale = gyro_scale
    
    def gyro_scale_raadout(self):
        '''
        ### Returns the gyro scale factor

        Use this when e.g. interacting with the base Inertial class (rather than the InertialWrapper class)
        '''
        return self.gyro_scale

    def gyro_scale_turn(self):
        '''
        ### Retunr inverse of gyro scale factor

        This is useful for correcting SmartDrive turns if SmartDrive is passed the base Inertial class
        '''
        return 1.0 / self.gyro_scale

    def rotation(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected rotation
        '''
        return super().rotation(units) * self.gyro_scale

    def heading(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected heading
        '''
        return InertialWrapper.to_heading(self.rotation(units))

    def angle(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected angle
        '''
        return InertialWrapper.to_angle(self.rotation(units))
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### Sets the Inertial rotation to the value specified while correcting for scale factor
        '''
        if units is not RotationUnits.DEG:
            raise NotImplementedError("InertialWrapper.set_rotation(): Units must be in degrees")
        rotation = value / self.gyro_scale
        return super().set_rotation(rotation, RotationUnits.DEG)
    
    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### Sets the Inertial heading to the value specified while correcting for scale factor
        '''
        if units is not RotationUnits.DEG:
            raise NotImplementedError("InertialWrapper.set_rotation(): Units must be in degrees")
        angle = InertialWrapper.to_angle(value)
        return self.set_rotation(angle, RotationUnits.DEG)

    @staticmethod
    def to_heading(rotation):
        '''
        ### Helper function to reduce a continues rotation value to a heading

        Used when you want to convert the raw rotation reading that can be in the range [-inf, +inf] \\
        to an absolute heading in the range [0, 360) degrees
        '''
        return rotation % 360.0

    @staticmethod
    def to_angle(rotation):
        '''
        ### Helper function to reduce a continues rotation value to an angle

        Used when you want to convert the raw rotation reading that can be in the range [-inf, +inf] \\
        to an absolute angle in the range (-180, 180] degrees
        '''
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle
    
    def calc_angle_to_rotation(self, rotation):
        '''
        ### Helper function to calculate the angle to an absolute rotation

        Used when you want to determine how far the robot needs to turn to point to a given rotation. \\
        Return value is an angle in the range [-180, 180]. This assumes you want to turn the shortest \\
        distance to the rotation (ie not the long way around)

        ### Arguments
            rotation: in degrees

        ### Returns
            Angle to rotation
        '''
        current_rotation = self.rotation()
        delta_rotation = rotation - current_rotation
        return delta_rotation

    def calc_angle_to_heading(self, heading):
        '''
        ### Helper function to calculate the angle to an absolute heading

        Used when you want to determine how far the robot needs to turn to point to a given heading. \\
        Return value is an angle in the range [-180, 180]. This assumes you want to turn the shortest \\
        distance to the heading (ie not the long way around)

        ### Arguments
            heading: in degrees

        ### Returns
            Angle to heading
        '''
        current_heading = self.heading()
        delta_heading = heading - current_heading
        delta_angle = InertialWrapper.to_angle(delta_heading)
        return delta_angle

    def calc_rotation_at_heading(self, heading):
        '''
        ### Helper function to calculate the inertial sensor rotation value for a given heading

        As the robot turns, the rotation will keep increasing, unlike heading that resets to zero every \\
        360 degrees. This means that to determine the sensor reading for a given heading we will need \\
        to back-calculate the rotation value

        ### Arguments
            heading: in degrees

        ### Returns
            Sensor rotation reading at that heading
        '''
        current_rotation = self.rotation()
        current_heading = InertialWrapper.to_heading(current_rotation)

        delta_heading = heading - current_heading
        delta_angle = InertialWrapper.to_angle(delta_heading)

        new_rotation = current_rotation + delta_angle

        return new_rotation
