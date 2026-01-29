from vex import *
from motorgroup import * # pyright: ignore[reportMissingImports]
import array

class Logger:
    '''
    Smart data capture for VEX devices and optional user-defined data fields. Logs data to the Brain's SD card in CSV format.\\
    Currently supports Motor, MotorGroup, Inertial, and Rotation devices.
    '''

    MOTOR = 1
    MOTOR_GROUP = 2
    INERTIAL = 3
    ROTATION = 4

    DEFAULT_LENGTH = 1000

    def __init__(self, brain: Brain, devices: List, headers: List, data_headers: List | None = None, data_fields_callback: Callable | None = None, max_length: int = -1, time_sec: int = -1, auto_dump: bool = False, file_name: str = "log"):
        '''
        ### Smart data capture for VEX devices and optional user-defined data fields.

        Logs data to the Brain's SD card in CSV format. Currently supports Motor, MotorGroup, Inertial, and Rotation devices.

        ### Device Logging

        Devices are specified as a list of instances, e.g. [motor1, interial1]. These are followed by a list of headers for each device, e.g. ["motor1", "inertial1"].

        Devices are automatically polled on average every 10ms, and data is logged when a change in timestamp is detected.
        - The position value in TURNS or REVOLUTIONS is captured for motors, motor groups and rotation sensors.
        - The rotation value in DEGREES is captured for inertial sensors.

        Each value is logged along with its timestamp in milliseconds.

        ### User-defined Data Fields

        Optional user-defined data fields can be logged by providing a list of data field headers, e.g. ["custom1", "custom2"],
        along with a callback function that returns a tuple of float values for each data field, e.g.

        def get_custom_data(): return (1.0, 2.0)

        This callback is expected to be efficient as it will be called every 10ms. No timestamp is logged for user-defined data fields -
        it gets called when any of the device timestamps change.

        ### Example CSV format:

        motor1_value, motor1_time, inertial1_value, inertial1_time, custom1, custom2\\
        1.5, 1000, 90.0, 1005, 3.14, 2.71\\
        1.7, 1010, 91.0, 1015, 3.15, 2.72\\
        ...

        ### Logging Duration

        By default the logger captures 1000 entries (approximately 10 seconds of data at 10ms intervals). This can be overridden by specifying either a maximum number of entries via max_length,
        or a maximum time duration via time_sec. If both are specified, max_length takes precedence.

        Logging is performed as lightweight as possible. If auto_dump is specified, then the logged data will be automatically saved to the SD card when the internal buffer is full or the time limit is reached.
        SD card access is noticeable during during opertion as it slows down IO calls, so if you only want to log a portion of the run, you
        can stop the logger manually when convenient at which point it will then dump the data to the SD card.

        ### SD Card Saving

        Saving to SDCard for a 60sec run will take about 10 seconds. Best way of capturing 60sec auton therefore is to start the program
        using the VEXNet Field Switch which keeps the program running after the auton ends. Using the Competition->Program Skills mode
        on the controller will stop the program immediately after auton ends which will not allow enough time to save the log data for a
        60 second run.
        
        ### Parameters

        :param brain: The Brain instance to use for logging, provides access to the SD card
        :type brain: Brain
        :param devices: List of devices to log, currently supports Motor, MotorGroup, Inertial, Rotation, e.g. [motor1, inertial1]
        :type devices: List
        :param headers: List of strings that will be used as headers for each column of the .csv file, e.g. ["motor1", "inertial1"]
        :type headers: List
        :param data_headers: List of strings for user-defined data fields to log, e.g. ["custom1", "custom2"]
        :type data_headers: List | None
        :param data_fields_callback: Callback function that returns a list of float values for the user-defined data fields as a tuple,
            e.g. def get_custom_data(): return (1.0, 2.0)
            Expect this to be called every 10ms, so keep it efficient
        :type data_fields_callback: Callable | None
        :param (optional) max_length: Overrides the seconds parameter to set the maximum number of data rows to log. Default is -1 (disabled).
        :type max_length: int
        :param (optional) time_sec: Optional maximum time in seconds to log data. Default is -1 (disabled).
            If neither max_length nor time_sec is specified, the default length of 1000 entries is used which corresponds to approximately 10 seconds of data
        :type time_sec: int
        :param auto_dump: If True, this will automatically dump the logged data to the SD card when the internal buffer is full or time limit is reached
        :type auto_dump: bool
        :param file_name: Optional base file name to use when dumping log data to SD card.
            A unique file name will be generated by appending an incrementing number up to 100. Default is "log", so file name is log1.csv, log2.csv, etc.
        :type file_name: str
        :return: New Logger instance
        :rtype: Logger

        ### Example Usage

        ```python
        from vex import *
        from logger import Logger

        brain = Brain()
        motor1 = Motor(Ports.PORT1)
        inertial1 = Inertial(Ports.PORT2)

        def get_custom_data():
            # Return some custom data values
            return (3.14, 2.71)

        logger = Logger(brain,
                        devices=[motor1, inertial1],
                        headers=["motor1", "inertial1"],
                        data_headers=["custom1", "custom2"],
                        data_fields_callback=get_custom_data,
                        time_sec=60,
                        auto_dump=True,
                        file_name="auton_log")

        logger.start()
        # Run your autonomous code here
        wait(60, SECONDS)  # Simulate 60 seconds of autonomous operation
        logger.stop() # optional stop

        ```
        '''

        # save brain and devices
        self.brain = brain
        if brain is None or not isinstance(brain, Brain):
            raise ValueError("Logger: A valid Brain instance must be provided")
        
        self.devices = devices
        self.headers = headers
        self.data_headers = data_headers
        self.data_fields_callback = data_fields_callback

        # figure out buffer lengths
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

        # pre-allocate data storage
        self.vdata = array.array('f', [0.0] * (self.rows * self.cols))
        self.tdata = array.array('i', [0] * (self.rows * self.cols))
        self.dfdata = array.array('f')  # Initialize empty
        if self.data_headers is not None and len(self.data_headers) > 0:
            if data_fields_callback is None:
                raise ValueError("Logger: data_fields_callback must be provided if data_fields is specified")
            self.dfcols = len(self.data_headers)
            self.dfdata = array.array('f', [0.0] * (self.rows * self.dfcols))
        else:
            self.data_fields_callback = None

        # need to preallocation bytearray for file writing, or we risk not being able to do this later
        self.bytearray = bytearray(4000)  # 4KB buffer for file writing
        self.bytelength = 4000

        # header line for .csv file
        text_buffer = ""
        header_line = ", ".join(ext for h in self.headers for ext in ("{}_value".format(h), "{}_time".format(h)) )
        text_buffer += header_line
        if self.dfcols > 0 and self.data_headers is not None:
            text_buffer += ", "
            df_header_line = ", ".join(h for h in self.data_headers)
            text_buffer += df_header_line
        text_buffer += "\n"
        self.header_buffer = bytearray(text_buffer, 'utf-8')

        # flags
        self.index = 0
        self.enabled = False
        self.thread = None
        self.auto_dump = auto_dump
        self.file_name = file_name
        self.saved = False

        # figure out device types and assign as an enum
        self.types = []
        for device in devices:
            if isinstance(device, Motor):
                self.types.append(Logger.MOTOR)
            elif isinstance(device, Inertial):
                self.types.append(Logger.INERTIAL)
            elif isinstance(device, Rotation):
                self.types.append(Logger.ROTATION)
            # BUGBUG: Motor groups don't seem to work when in separate file
            # elif isinstance(device, MotorGroup):
            elif type(device) is MotorGroup:
                self.types.append(Logger.MOTOR_GROUP)
            else:
                raise ValueError("Unsupported device type")

    def start(self):
        '''
        ### Start logging data
        '''
        if (not self.enabled):
            self.enabled = True
            self.thread = Thread(self._log_thread)

    def stop(self, dump: bool = False):
        '''
        ### Stop logging data

        Optional call to dump the logged data before the internal buffer is full or the time limit is reached.

        Note that if dump parameter is not specified, and auto_dump was set to True during initialization, the data will NOT be saved

        :param dump: If True, will dump the logged data to the SD card
        :type dump: bool
        '''
        if (self.enabled):
            self.enabled = False
            self.auto_dump = dump

    def dump(self):
        '''
        ### Dump logged data to the Brain's SD card
        '''
        # TODO: check if enabled and stop first?
        print("Logger: Dumping logged data to SD card")
        
        if self.saved:
            print("Logger: Data has already been dumped to SD card")
            return False
        
        if self.enabled:
            print("Logger: Stopping logging before dump")
            # in this case let stop() handle the dump so things wrap up properly
            self.stop(True)
            return True

        # if logging has already stopped and has not already been saved, manually dump here
        self._dump(self.file_name)
        return True

    def print(self, rate: int = 100):
        '''
        ### Print logged data to the Brain screen

        :param rate: Rate in milliseconds to print each row of data
        :type rate: int
        '''
        print("Logger: Printing logged data")
        for i in range(self.index):
            line = ""
            for j in range(self.cols):
                val = self.vdata[i * self.cols + j]
                time = self.tdata[i * self.cols + j]
                line += "{:.2f}, {}, ".format(val, time)
            if self.dfcols > 0:
                for k in range(self.dfcols):
                    dfval = self.dfdata[i * self.dfcols + k]
                    line += "{:.2f}".format(dfval)
                    if k < self.dfcols - 1:
                        line += ", "
            print(line)
            wait(rate, MSEC)

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

                if self.dfcols > 0 and self.data_fields_callback is not None:
                    df_values = self.data_fields_callback()
                    # check we got valid values back
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
        self.saved = True
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

        #self.vdata = None
        #self.tdata = None
        #self.bytearray = None

    def fileio_test(self):
        f = open("test.txt", "w")
        f.write("This is a test\n")
        f.flush()
        f.close()