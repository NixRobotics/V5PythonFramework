from vex import *
from motorgroup import * # pyright: ignore[reportMissingImports]
from drivetrain import * # pyright: ignore[reportMissingImports]
from smartdrive import * # pyright: ignore[reportMissingImports]
from inertialwrapper import InertialWrapper

class SmartDriveWrapper(SmartDrive):
    # @param port
    # @param gyro_scale: GYRO_SCALE_FOR_FOR_READOUT
    def __init__(self,
                 lm: MotorGroup,
                 rm: MotorGroup,
                 g: InertialWrapper,
                 wheelTravel = 300.0,
                 trackWidth = 320.0,
                 wheelBase = 320.0,
                 units = DistanceUnits.MM,
                 externalGearRatio = 1.0):
        
        self.g = g
        
        super().__init__(lm, rm, g, wheelTravel, trackWidth, wheelBase, units, externalGearRatio)

    def turn_for(self, direction, angle, units = RotationUnits.DEG,
                 velocity=None, units_v:VelocityPercentUnits = VelocityUnits.RPM, wait=True):
        return super().turn_for(direction, angle, units, velocity, units_v, wait)

    def drive_for(self, direction, distance, units = DistanceUnits.IN,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.RPM, wait=True):
        return super().drive_for(direction, distance, units, velocity, units_v, wait)

    def drive_straight(self, direction, distance, units = DistanceUnits.IN,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.RPM,
                  heading = None, units_h = RotationUnits.DEG,
                  wait=True):
        return super().drive_for(direction, distance, units, velocity, units_v, wait)
