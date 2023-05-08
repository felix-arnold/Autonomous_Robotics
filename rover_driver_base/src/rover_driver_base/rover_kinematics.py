#!/usr/bin/env python
#import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import lstsq
from numpy import mod
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True
        self.last_drive = {}
        for k in prefix:
            self.last_drive[k]=0.0

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                vx = twist.linear.x - twist.linear.y * drive_cfg[k].y
                motors.steering[k] = 0
                motors.drive[k] = vx/drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                vx = twist.linear.x - twist.angular.z * drive_cfg[k].y
                vy = twist.linear.y + twist.angular.z * drive_cfg[k].x
                if (abs(drive_state.steering[k] - atan2(vy, vx)) > pi/2):
                    motors.steering[k] = atan2(vy, vx)-pi
                    motors.drive[k] = -hypot(vy, vx)/drive_cfg[k].radius
                else :
                    motors.steering[k] = atan2(vy, vx)
                    motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return self.X
        # Insert here your odometry code
        A = numpy.asmatrix(numpy.zeros((12,3)))
        B = numpy.asmatrix(numpy.zeros((12,1)))
        i=0
        for k in drive_cfg.keys():
            A[i,0],   A[i,1],   A[i,2]   = 1, 0, -drive_cfg[k].y
            A[i+1,0], A[i+1,1], A[i+1,2] = 0, 1, drive_cfg[k].x
            s = (mod(motor_state.drive[k] - self.last_drive[k] + pi, 2*pi) - pi) * drive_cfg[k].radius 
            B[i,0]   = s * cos(motor_state.steering[k])
            B[i+1,0] = s * sin(motor_state.steering[k])
            self.last_drive[k] = motor_state.drive[k]
            i+=2
        D = lstsq(A, B)
        self.X[0,0] += cos(self.X[2,0])*D[0][0] - sin(self.X[2,0])*D[0][1]
        self.X[1,0] += sin(self.X[2,0])*D[0][0] + cos(self.X[2,0])*D[0][1]
        self.X[2,0] += D[0][2]
        self.motor_state.copy(motor_state)
        return self.X
    



