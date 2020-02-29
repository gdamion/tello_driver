# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importing time for time management
import time


class PID(object):

    def __init__(self, coeff_p, coeff_i, coeff_d, anti_windup):

        self.goal_pose = goal_pose
        self.current_pose = current_pose
        # p = 0.8 i = 0.0001 d = 0.003
        self.coeff_p = coeff_p
        self.coeff_i = coeff_i
        self.coeff_d = coeff_d

        self.anti_windup = anti_windup

        self.pose_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.pose_error_old = 0

        self.control = 0


    def updatePidControl(self, goal_pose, current_pose, dt):

        # drone errors
        self.pose_error = goal_pose - current_pose

        self.integral_error += dt * (self.pose_error + self.pose_error_old)/2
        self.integral_error = self.relu(integral_error, self.anti_windup)

        self.derivative_error = (self.pose_error_old - self.pose_error)/dt

        self.pose_error_old = self.pose_error

        self.control = self.coeff_p * self.pose_error + self.coeff_i * self.integral_error +
        self.coeff_d * self.derivative_error

        return control

    def relu(self, input, value):
        if input > value:
            input = value
        if input < -value:
            input = -value
        return input



    while(True):
