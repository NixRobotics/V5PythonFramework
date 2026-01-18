from vex import *

class Logger:

    MOTOR = 1
    INERTIAL = 2
    ROTATION = 3

    DEFAULT_LENGTH = 1000

    def __init__(self, brain: Brain, devices: List, headers: List, max_length: int = -1, time_sec: int = -1, auto_dump: bool = False, file_name: str = "log"):
        self.brain = brain
        if brain is None or not isinstance(brain, Brain):
            raise ValueError("Logger: A valid Brain instance must be provided")
        self.devices = devices
        self.headers = headers
        if max_length > 0:
            self.rows = max_length
        if time_sec > 0:
            self.timeout = time_sec * 1000  # Convert to milliseconds
            if max_length == -1:
                self.rows = self.timeout // 10  # Assuming new logging value every 10ms
        else:
            self.timeout = -1
        self.rows = max_length if max_length != -1 else Logger.DEFAULT_LENGTH
        self.cols = 2 * len(devices)  # Each device has value and timestamp
        self.data = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.index = 0
        self.enabled = False
        self.thread = None
        self.auto_dump = auto_dump
        self.file_name = file_name
        self.types = []
        for device in devices:
            if isinstance(device, Motor):
                self.types.append(Logger.MOTOR)
            elif isinstance(device, Inertial):
                self.types.append(Logger.INERTIAL)
            elif isinstance(device, Rotation):
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
        print("Logger: Logging started")
        timer = Timer()
        start_time = timer.time(MSEC)
        while self.enabled:
            if self.timeout > 0 and (timer.time(MSEC) - start_time) >= self.timeout:
                self.enabled = False
                break
            updated = False
            i = 0
            for device in self.devices:
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
                i += 1
            if updated:
                if self.index < self.rows - 1:
                    self.index = (self.index + 1) % self.rows
                else:
                    self.enabled = False
                    break
            wait(5, MSEC)  # Check every 5ms

        if self.auto_dump:
            self._dump(self.file_name)

    def _dump(self, file_name: str = "log"):
        print("Logger: Dumping log data to SD card")
        if not self.brain.sdcard.is_inserted():
            raise RuntimeError("Logger: No SD card inserted in Brain")
        
        full_file_name = None
        for attempt in range(100):
            try_name = "{}_{}.csv".format(file_name, attempt)
            if not self.brain.sdcard.exists(try_name):
                full_file_name = try_name
                break

        if full_file_name is None:
            raise RuntimeError("Logger: Could not create unique log file name")
        
        text_buffer = "%1.3f" % self.brain.timer.value() + "\n"
        byte_buffer = bytearray(text_buffer, 'utf-8')
        bytes_written = self.brain.sdcard.savefile(full_file_name, byte_buffer)

        if bytes_written <= 0:
            raise RuntimeError("Logger: Failed to write log data to SD card")

        print("Logger: Log data saved to {}, length {}".format(full_file_name, bytes_written))

        #with open(f"{file_name}.csv", "w") as f:
        #    header_line = ",".join(self.headers)
        #    f.write(header_line + "\n")
        #    for i in range(self.index):
        #        row_data = []
        #        for j in range(self.cols):
        #            row_data.append(str(self.data[i][j]))
        #        f.write(",".join(row_data) + "\n")
            

