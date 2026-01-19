from vex import *
from motorgroup import * # pyright: ignore[reportMissingImports]
import array

class Logger:

    MOTOR = 1
    MOTOR_GROUP = 2
    INERTIAL = 3
    ROTATION = 4

    DEFAULT_LENGTH = 1000

    def __init__(self, brain: Brain, devices: List, headers: List, data_headers: List = None, data_fields_callback: Callable = None, max_length: int = -1, time_sec: int = -1, auto_dump: bool = False, file_name: str = "log"):
        self.brain = brain
        if brain is None or not isinstance(brain, Brain):
            raise ValueError("Logger: A valid Brain instance must be provided")
        
        self.devices = devices
        self.headers = headers
        self.data_headers = data_headers
        self.data_fields_callback = data_fields_callback

        if max_length > 0:
            self.rows = max_length

        if time_sec > 0:
            self.timeout = time_sec * 1000  # Convert to milliseconds
            if max_length <= 0:
                self.rows = self.timeout // 10  # Assuming new logging value every 10ms
        else:
            self.timeout = -1

        if (max_length <= 0 and time_sec <= 0):
            self.rows = Logger.DEFAULT_LENGTH

        self.cols = len(devices)  # Each device has value and timestamp
        self.dfcols = 0

        self.vdata = array.array('f', [0.0] * (self.rows * self.cols))
        self.tdata = array.array('i', [0] * (self.rows * self.cols))
        self.dfdata = None
        if self.data_headers is not None and len(self.data_headers) > 0:
            if data_fields_callback is None:
                raise ValueError("Logger: data_fields_callback must be provided if data_fields is specified")
            self.dfcols = len(self.data_headers)
            self.dfdata = array.array('f', [0.0] * (self.rows * self.dfcols))
        else:
            self.data_fields_callback = None

        self.bytearray = bytearray(4000)  # 4KB buffer for file writing
        self.bytelength = 4000

        text_buffer = ""
        header_line = ", ".join(ext for h in self.headers for ext in ("{}_value".format(h), "{}_time".format(h)) )
        text_buffer += header_line
        if self.dfcols > 0:
            text_buffer += ", "
            df_header_line = ", ".join(h for h in self.data_headers)
            text_buffer += df_header_line
        text_buffer += "\n"
        self.header_buffer = bytearray(text_buffer, 'utf-8')

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
            # BUGBUG: Motor groups don't seem to work when in separate file
            elif isinstance(device, MotorGroup):
                self.types.append(Logger.MOTOR_GROUP)
            else:
                raise ValueError("Unsupported device type")

    def start(self):
        if (not self.enabled):
            self.enabled = True
            self.thread = Thread(self._log_thread)

    def stop(self, dump: bool = False):
        if (self.enabled):
            self.enabled = False
            self.auto_dump = dump

    def _avg_motor_times(self, mg: MotorGroup):
        '''
        ### INTERNAL Docstring for avg_motor_times
        
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

    def _read_motor_group(self, motor_group: MotorGroup):
        return motor_group.position(TURNS), int(self._avg_motor_times(motor_group))

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
        values = [0.0] * len(self.devices)
        timestamps = [0] * len(self.devices)
        last_timestamps = [0] * len(self.devices)

        while self.enabled:
            if self.timeout > 0 and (timer.time(MSEC) - start_time) >= self.timeout:
                self.enabled = False
                break

            updated = False

            for i in range(self.cols):
                if self.types[i] == Logger.MOTOR:
                    values[i], timestamps[i] = self._read_motor(self.devices[i])
                elif self.types[i] == Logger.INERTIAL:
                    values[i], timestamps[i] = self._read_inertial(self.devices[i])
                elif self.types[i] == Logger.ROTATION:
                    values[i], timestamps[i] = self._read_rotation(self.devices[i])
                elif self.types[i] == Logger.MOTOR_GROUP:
                    values[i], timestamps[i] = self._read_motor_group(self.devices[i])
                if timestamps[i] != last_timestamps[i]:
                    updated = True

            if updated:
                for i in range(self.cols):
                    self.vdata[self.index * self.cols + i] = values[i]
                    self.tdata[self.index * self.cols + i] = timestamps[i]
                    last_timestamps[i] = timestamps[i]

                if self.dfcols > 0:
                    df_values = self.data_fields_callback()
                    if (df_values is None) or (len(df_values) != self.dfcols):
                        raise RuntimeError("Logger: data_fields_callback did not return expected number of values")
                    for j in range(self.dfcols):
                        self.dfdata[self.index * self.dfcols + j] = df_values[j]

                if self.index < self.rows - 1:
                    self.index = (self.index + 1) % self.rows
                else:
                    self.enabled = False
                    break

            wait(5, MSEC)  # Check every 5ms

        if self.auto_dump:
            self._dump(self.file_name)

    def _dump_buffer(self, full_file_name) -> int:
            bytes_written_this = self.brain.sdcard.appendfile(full_file_name, self.bytearray)
            if bytes_written_this <= 0:
                raise RuntimeError("Logger: Failed to write log data to SD card")
            # print("Logger: Wrote {} bytes to log file".format(bytes_written_this))
            return bytes_written_this

    def _write_buffer(self, full_file_name: str, pos: int, s:str) -> int:
        for c in s.encode('utf-8'):
            self.bytearray[pos] = c
            pos += 1
            if pos >= self.bytelength:
                self._dump_buffer(full_file_name)
                pos = 0

        return pos
    
    def _flush_buffer(self, full_file_name: str, pos: int) -> int:
        if pos > 0:
            self.bytearray = self.bytearray[0:pos]
            bytes_written_this = self.brain.sdcard.appendfile(full_file_name, self.bytearray)
            if bytes_written_this <= 0:
                raise RuntimeError("Logger: Failed to write log data to SD card")
            # print("Logger: Wrote {} bytes to log file".format(bytes_written_this))
            return bytes_written_this
        return 0

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
        
        bytes_written = self.brain.sdcard.savefile(full_file_name, self.header_buffer)
        if bytes_written <= 0:
            raise RuntimeError("Logger: Failed to write log data to SD card")
        
        lines_to_write = self.index
        read_index = 0
        print("Logger: Writing {} lines to log file".format(lines_to_write))

        pos = 0
        while (lines_to_write > 0):
            for col in range(self.cols):
                
                val = self.vdata[read_index * self.cols + col]
                s = str(val)
                pos = self._write_buffer(full_file_name, pos, s)
                pos = self._write_buffer(full_file_name, pos, ',')

                time =  self.tdata[read_index * self.cols + col]
                s = str(time)
                pos = self._write_buffer(full_file_name, pos, s)
                if (col < self.cols - 1 or self.dfcols > 0):
                   pos = self._write_buffer(full_file_name, pos, ',')
                else:
                   pos = self._write_buffer(full_file_name, pos, '\n')

            if self.dfcols > 0:
                for dfcol in range(self.dfcols):
                    dfval = self.dfdata[read_index * self.dfcols + dfcol]
                    s = str(dfval)
                    pos = self._write_buffer(full_file_name, pos, s)
                    if (dfcol < self.dfcols - 1):
                        pos = self._write_buffer(full_file_name, pos, ',')
                    else:
                        pos = self._write_buffer(full_file_name, pos, '\n')

            read_index += 1
            lines_to_write -= 1

        self._flush_buffer(full_file_name, pos)

        print("Logger: Log data saved to {}".format(full_file_name))

        self.vdata = None
        self.tdata = None
        self.bytearray = None

