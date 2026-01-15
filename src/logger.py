from vex import *

class Logger:

    MOTOR = 1
    INERTIAL = 2
    ROTATION = 3

    def __init__(self, devices: List, headers: List, length: int = 1000):
        self.devices = devices
        self.headers = headers
        self.rows = length
        self.cols = 2 * len(devices)  # Each device has value and timestamp
        self.data = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.index = 0
        self.enabled = False
        self.thread = None
        self.types = []
        for device in devices:
            if device is Motor:
                self.types.append(Logger.MOTOR)
            elif device is Inertial:
                self.types.append(Logger.INERTIAL)
            elif device is Rotation:
                self.types.append(Logger.ROTATION)
            else:
                raise ValueError("Unsupported device type")

    def start(self):
        if (not self.enabled):
            self.enabled = True
            self.thread = Thread(self._log_thread)

    def stop(self):
        if (self.enabled):
            self.enabled = False

    def _read_motor(self, motor: Motor):
        return motor.position(TURNS), motor.timestamp()

    def _read_inertial(self, inertial: Inertial):
        return inertial.rotation(DEGREES), inertial.timestamp()
    
    def _read_rotation(self, rotation: Rotation):
        return rotation.position(DEGREES), rotation.timestamp()

    def _log_thread(self):
        while self.enabled:
            updated = False
            for i, device in enumerate(self.devices):
                if self.types[i] == Logger.MOTOR:
                    this_value, this_timestamp = self._read_motor(device)
                elif self.types[i] == Logger.INERTIAL:
                    this_value, this_timestamp = self._read_inertial(device)
                elif self.types[i] == Logger.ROTATION:
                    this_value, this_timestamp = self._read_rotation(device)
                last_timestamp = self.data[self.index - 1][2*i + 1] if self.index > 0 else -1
                if this_timestamp != last_timestamp:
                    updated = True
                    self.data[self.index][2*i] = this_value
                    self.data[self.index][2*i + 1] = this_timestamp
            if updated:
                if self.index < self.rows - 1:
                    self.index = (self.index + 1) % self.rows
            wait(5, MSEC)  # Check every 5ms
            