from math import atan2, sin, cos

class PidController:
    def __init__(self, Kp, Ki, Kd, delta_time, is_angle):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.delta_time = delta_time
        self.is_angle = is_angle

        self.out_min = 0
        self.out_max = 1
        self.input = 0
        self.setpoint = 0
        self.output = 0

        self.error = 0
        self.prev_error = 0
        self.sum_error = 0

    def set_output_limits(self, min, max):
        self.out_min = min
        self.out_max = max

    def set_input(self, new_input):
        self.input = new_input

    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint

    def get_output(self):
        return self.output

    def compute(self):
        self.error =  self.setpoint - self.input

        if self.is_angle:
            self.error = atan2(sin(self.error), cos(self.error))

        P = self.Kp * self.error
        I = self.sum_error + self.Ki * self.error * self.delta_time
        D = self.Kd * (self.error - self.prev_error) / self.delta_time

        self.output += P + I + D

        if self.output > self.out_max:
            self.output = self.out_max
        if self.output < self.out_min:
            self.output = self.out_min

        self.prev_error = self.error
        self.sum_error = I
