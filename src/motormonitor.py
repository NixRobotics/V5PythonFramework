from vex import *

# Monitor Monitoring
class MotorMonitor:

    # motors will slow down above certain temperatures
    # vex defines warm as 50 percent and hot as 70 percent
    MOTOR_HOT_TEMP = 70
    MOTOR_WARM_TEMP = 60
    MOTOR_COOL_TEMP = 50

    MOTOR_STATUS_OK = 0
    MOTOR_STATUS_NOT_PRESENT = 1
    MOTOR_STATUS_COOL = 2
    MOTOR_STATUS_WARM = 3
    MOTOR_STATUS_HOT = 4

    def __init__(self, brain: Brain, allmotors: List[Motor], motor_names: List[str]):
        '''
        Docstring for __init__
        
        :param brain: Description
        :type brain: Brain
        :param allmotors: Description
        :type allmotors: List[Motor]
        :param motor_names: Description
        :type motor_names: List[str]
        '''
        self.brain = brain # type: Brain
        self.allmotors = allmotors
        self.motor_names = motor_names
        self.motor_status = [MotorMonitor.MOTOR_STATUS_OK]  * len(allmotors)
        self.previous_motor_status = self.motor_status.copy()
        self.motor_status_names = ["OK", "NOT PRESENT", "OK FOR NOW", "WARM", "HOT"]
        self.motor_monitor_refresh_UI = False

    def start(self):
        '''
        Docstring for start
        '''
        self.thread = Thread(self._motor_monitor_thread, (self.brain, self.allmotors, self.motor_names))

    def refresh(self):
        '''
        Docstring for refresh
        '''
        self.motor_monitor_refresh_UI = True

    # this function checks the temperature of each motor and then returns two values: warm, hot
    def _get_motor_status(self):
        # to make code simple we create a list of all the motors, then check each one
        motor_error = False
        motor_is_cool = False
        motor_is_warm = False
        motor_is_hot = False
        for i in range(len(self.allmotors)):
            motor = self.allmotors[i] # type: Motor
            self.motor_status[i] = MotorMonitor.MOTOR_STATUS_OK # assume motor is ok
            if (not motor.installed()):
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_NOT_PRESENT
                motor_error = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_HOT_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_HOT
                motor_is_hot = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_WARM_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_WARM
                motor_is_warm = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_COOL_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_COOL
                motor_is_cool = True

        is_changed = self.motor_status != self.previous_motor_status
        self.previous_motor_status = self.motor_status.copy()

        return is_changed, motor_error, motor_is_cool, motor_is_warm, motor_is_hot

    # we run this in its own thread to monitor the temperature each second and change the color of the screen
    # green is first warning
    # blue is warm
    # red is hot
    def _monitor_UI(self, motor_error, motor_is_cool, motor_is_warm, motor_is_hot):

        if (motor_is_hot): self.brain.screen.clear_screen(Color.RED)
        elif (motor_is_warm): self.brain.screen.clear_screen(Color.BLUE)
        elif (motor_is_cool): self.brain.screen.clear_screen(Color.GREEN)
        else: self.brain.screen.clear_screen(Color.BLACK)

        if (motor_error or motor_is_hot or motor_is_warm or motor_is_cool):
            self.brain.screen.set_cursor(1,1)
            self.brain.screen.set_font(FontType.MONO20)
            self.brain.screen.set_fill_color(Color.BLACK)
            self.brain.screen.set_pen_color(Color.WHITE)
            for i in range(len(self.allmotors)):
                if (self.motor_status[i] != MotorMonitor.MOTOR_STATUS_OK):
                    self.brain.screen.print(self.motor_names[i] + ": " + self.motor_status_names[self.motor_status[i]])
                    self.brain.screen.new_line()
            # brain.screen.render()

    def _motor_monitor_thread(self, brain: Brain, allmotors: List[Motor], motor_names: List[str]):
        monitor = MotorMonitor(brain, allmotors, motor_names)
        while(True):
            is_changed, motor_error, motor_is_cool, motor_is_warm, motor_is_hot = monitor._get_motor_status()
            # only call UI routines if status has changed
            if (is_changed or self.motor_monitor_refresh_UI):
                monitor._monitor_UI(motor_error, motor_is_cool, motor_is_warm, motor_is_hot)
                self.motor_monitor_refresh_UI = False
            wait(2, TimeUnits.SECONDS)
