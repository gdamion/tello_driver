#!/usr/bin/env python2
# coding: utf-8

import rospy
from numpy import *
import threading
import traceback
import time
import sys
import tf.transformations as tftr
import math

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, Point, Vector3, Twist
from sensor_msgs.msg import CameraInfo, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from tello_driver.msg import TelloStatus
from dynamic_reconfigure.server import Server
from tello_driver.cfg import TelloConfig

lock = threading.Lock()

class DroneController:

    def __init__(self):
	    #ROS PUB
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
        self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1, latch=True)
        self.fast_mode_pub = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1, latch=True)
        self.flattrim_pub = rospy.Publisher('/tello/flattrim', Empty, queue_size=1, latch=True)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1, latch=True)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1, latch=True)
        self.manual_takeoff_pub = rospy.Publisher('/tello/manual_takeoff', Empty, queue_size=1, latch=True)

        #ROS SUB
        self.odom_sub = rospy.Subscriber('/tello/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/tello/imu', Imu, self.imu_callback)
        self.status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.status_callback)

        #DRONE STATE
        self.imu = Imu()
        self.status = TelloStatus()

        self.state_position = Point()
        self.state_orientation = Vector3()
        self.state_lin_vel = Vector3()
        self.state_ang_vel = Vector3()

        #CALL START METHOD


    def odom_callback(self, msg):
        lock.acquire()
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.state_orientation = Vector3(roll, pitch, yaw)
        self.state_lin_vel = msg.twist.twist.linear
        self.state_ang_vel = msg.twist.twist.angular
        lock.release()

    def imu_callback(self, msg):
        lock.acquire()
        self.imu = msg #SAVE FRESH IMU
        lock.release()

    def status_callback(self, msg):
        lock.acquire()
        self.status = msg #SAVE FRESH STATUS
        lock.release()

    def stop(self):
        """ Reset the robot """
        msg = Empty()
        self.land_pub.publish(msg)

        self.odom_sub.unregister()
        self.imu_sub.unregister()
        self.status_sub.unregister()


def main():
    control = DroneController()

    ####

        self.stop()

    def emergency_stop(self):
    	print("Emergency stop")
    	self.emergency_pub.publish(Empty)
    
    def update():
    	#main loop
    	while not rospy.is_shutdown():
    		


main = None
    while not rospy.is_shutdown():
        try:
            main = DroneController()
            main.update()
        except rospy.ROSInterruptException as e:
            #main.close()
            main.emergency_stop()
            del main
            print('End')
