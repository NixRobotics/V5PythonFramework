# Port from https://github.com/tekdemo/MiniPID

# Library imports
from vex import *
from math import pi

class PID:
    def __init__(self, Kp, Ki, Kd, Kf):
        self.init()

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kf = Kf
        self.F_offset = 0

    def init(self):
        self.maxIOutput = 0
        self.maxError = 0
        self.errorSum = 0
        self.maxOutput = 0 
        self.minOutput = 0
        self.setpoint = 0
        self.lastActual = 0
        self.firstRun = True
        self.reversed = False
        self.outputRampRate = 0
        self.lastOutput = 0
        self.outputFilter = 0
        self.setpointRange = 0

    def setMaxIOutput(self, maximum):
        self.maxIOutput = maximum
        if (self.Ki != 0):
            self.maxError = self.maxIOutput / self.Ki

    def setOutputLimits(self, minimum, maximum):
        if (maximum < minimum): return
        self.maxOutput = maximum
        self.minOutput = minimum

        # Ensure the bounds of the I term are within the bounds of the allowable output swing
        if (self.maxIOutput == 0 or self.maxIOutput > (maximum - minimum)):
            self.setMaxIOutput(maximum - minimum)

    def setDirection(self, reversed):
        self.reversed = reversed

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def getOutput(self, actual = None, setpoint = None):
        if actual == None: actual = self.lastActual
        if setpoint == None: setpoint = self.setpoint

        output = 0
        Poutput = 0
        Ioutput = 0
        Doutput = 0
        Foutput = 0

        self.setpoint = setpoint

        # Ramp the setpoint used for calculations if user has opted to do so
        if (self.setpointRange != 0):
            setpoint = self.clamp(setpoint, actual - self.setpointRange, actual + self.setpointRange)

        # Do the simple parts of the calculations
        error = setpoint - actual

        # Calculate F output. Notice, this depends only on the setpoint, and not the error.
        if (setpoint != 0.0): F_offset_this = self.F_offset
        else: F_offset_this = 0.0
        Foutput = self.Kf * (setpoint + F_offset_this)

        # Calculate P term
        Poutput = self.Kp * error;	 

        # If this is our first time running this we don't actually _have_ a previous input or output. 
        # For sensor, sanely assume it was exactly where it is now.
        # For last output, we can assume it's the current time-independent outputs. 
        if (self.firstRun):
            self.lastActual = actual
            self.lastOutput = Poutput + Foutput
            self.firstRun = False

        # Calculate D Term
        # Note, this is negative. this actually "slows" the system if it's doing
        # the correct thing, and small values helps prevent output spikes and overshoot 

        Doutput = -self.Kd * (actual - self.lastActual)
        self.lastActual = actual

        # The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        # 1. maxIoutput restricts the amount of output contributed by the Iterm.
        # 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        # 3. prevent windup by not increasing errorSum if output is output=maxOutput	
        Ioutput = self.Ki * self.errorSum
        if (self.maxIOutput != 0):
            Ioutput = self.clamp(Ioutput, -self.maxIOutput, self.maxIOutput) 

        # And, finally, we can just add the terms up
        output = Foutput + Poutput + Ioutput + Doutput

        # Figure out what we're doing with the error.
        if (self.minOutput != self.maxOutput and not self.bounded(output, self.minOutput, self.maxOutput) ):
            self.errorSum = error
            # reset the error sum to a sane level
            # Setting to current error ensures a smooth transition when the P term 
            # decreases enough for the I term to start acting upon the controller
            # From that point the I term will build up as would be expected
        elif(self.outputRampRate != 0 and not self.bounded(output, self.lastOutput - self.outputRampRate, self.lastOutput + self.outputRampRate)):
            self.errorSum = error

        elif(self.maxIOutput != 0):
            self.errorSum = self.clamp(self.errorSum + error, -self.maxError, self.maxError)
            # In addition to output limiting directly, we also want to prevent I term 
            # buildup, so restrict the error directly
        else:
            self.errorSum += error

        # Restrict output to our specified output and ramp limits
        if (self.outputRampRate != 0):
            output = self.clamp(output, self.lastOutput - self.outputRampRate, self.lastOutput + self.outputRampRate)
        if (self.minOutput != self.maxOutput): 
            output = self.clamp(output, self.minOutput, self.maxOutput)
        if (self.outputFilter != 0):
            output = self.lastOutput * self.outputFilter + output * (1.0 - self.outputFilter)

        self.lastOutput = output
        return output
    
    def reset(self):
        self.firstRun = True
        self.errorSum = 0

    def clear(self):
        self.errorSum = 0.0
        self.lastActual = 0.0

    def setOutputRampRate(self, rate):
        self.outputRampRate = rate

    def setSetpointRange(self, range):
        self.setpointRange = range

    def setOutputFilter(self, strength):
        if (strength == 0 or self.bounded(strength, 0, 1)):
            self.outputFilter = strength

    def clamp(self, value, min, max):
        if (value > max): return max
        if (value < min): return min
        return value

    def bounded(self, value, min, max):
        return (min < value) and (value < max)
 
