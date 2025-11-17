from vex import Inertial, RotationUnits

class InertialWrapper(Inertial):

    # @param port
    # @param gyro_scale: GYRO_SCALE_FOR_FOR_READOUT
    def __init__(self, port, gyro_scale):
        super().__init__(port)
        self.gyro_scale = gyro_scale
    
    def gyro_scale_raadout(self):
        return self.gyro_scale

    def gyro_scale_turn(self):
        return 1.0 / self.gyro_scale

    def rotation(self, units = RotationUnits.DEG):
        return super().rotation(units) * self.gyro_scale

    def heading(self, units = RotationUnits.DEG):
        return InertialWrapper.to_heading(self.rotation(units))

    def angle(self, units = RotationUnits.DEG):
        return InertialWrapper.to_angle(self.rotation(units))
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        if units is not RotationUnits.DEG:
            raise NotImplementedError("InertialWrapper.set_rotation(): Units must be in degrees")
        rotation = value / self.gyro_scale
        return super().set_rotation(rotation, RotationUnits.DEG)
    
    def set_heading(self, value, units=RotationUnits.DEG):
        if units is not RotationUnits.DEG:
            raise NotImplementedError("InertialWrapper.set_rotation(): Units must be in degrees")
        angle = InertialWrapper.to_angle(value)
        return self.set_rotation(angle, RotationUnits.DEG)

    @staticmethod
    def to_heading(rotation):
        return rotation % 360.0

    @staticmethod
    def to_angle(rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    def calc_angle_to_heading(self, heading):
        current_heading = self.heading()
        delta_heading = heading - current_heading
        delta_angle = InertialWrapper.to_angle(delta_heading) / self.gyro_scale # divide: sic
        return delta_angle

    def calc_rotation_at_heading(self, heading):
        current_rotation = self.rotation()
        current_heading = InertialWrapper.to_heading(current_rotation)

        delta_heading = heading - current_heading
        delta_angle = InertialWrapper.to_angle(delta_heading)

        new_rotation = current_rotation + delta_angle
        new_rotation *= (1.0 / self.gyro_scale)

        return new_rotation
