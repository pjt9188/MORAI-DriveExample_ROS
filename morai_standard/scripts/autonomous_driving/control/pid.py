#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from sys import float_info


class Pid:
    def __init__(self, p_gain, i_gain, d_gain, sampling_time):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.sampling_time = sampling_time

        self.previous_error = 0
        self.integral_error = 0

        self.start_integral = False

    def get_output(self, target_value, current_value):
        error = target_value-current_value
        
        # Initial excessive integration error 방지
        if (error <= target_value * 0.1) and not self.start_integral:
            self.start_integral = True
        
        # Integration error 누적 방지
        if self.start_integral == True:
            self.integral_error = error*self.sampling_time + self.integral_error*(1-self.sampling_time)
        
        derivative_error = (error-self.previous_error)/self.sampling_time

        print("rate : err = {}, Int err = {}, Der err ={}".format(error, self.integral_error, derivative_error))
        output = self.p_gain*error + self.i_gain*self.integral_error + self.d_gain*derivative_error
        self.previous_error = error
        return output
