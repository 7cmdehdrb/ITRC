import numpy as np


class LowPassFilter:
    def __init__(self, cutoff_freq, ts):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.pre_out = 0.0
        self.tau = self.calc_filter_coef()

    def calc_filter_coef(self):
        w_cut = 2 * np.pi * self.cutoff_freq
        return 1 / w_cut

    def filter(self, data):
        out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
        self.pre_out = out
        return out


class PID:
    def __init__(self, kp, ki, kd):
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # PID variables
        self.integral = 0
        self.prev_error = 0

    def update(self, desired_value: float, measured_value: float, dt: float) -> float:
        error = desired_value - measured_value

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error

        k_err = self.kp * error
        k_int = self.ki * self.integral
        k_der = self.kd * derivative

        return k_err + k_int + k_der
