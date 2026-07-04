
from pybricks.tools import StopWatch

class PID_values:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

class PID_control:
    def __init__(self, kp, ki, kd, max_ouput=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.error = 0

        self.integral_limit = integral_limit

        self.max_output = max_ouput

    def reset(self):
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0

    def compute(self, error):
        # Calculate Error
        self.error = error

        # Proportional
        self.proportional = self.error

        # Integral
        self.integral += self.error

        #Clamp Integral
        if self.integral_limit is not None and abs(self.integral) > self.integral_limit:
            self.integral = self.integral_limit * (1 if self.integral > 0 else -1)
            

        # Derivative
        derivative = self.error - self.last_error
        self.last_error = self.error

        # PID Output
        output = (
            self.kp * self.proportional +
            self.ki * self.integral +
            self.kd * self.derivative
        )

        return output