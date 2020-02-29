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
#from pid import *

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
        self.drone_state_position = msg.pose.pose.position
        self.drone_state_position.y *= -1
        self.drone_state_position.z *= -1

        self.state_orientation = Vector3(roll, -pitch, -yaw)
        self.state_lin_vel = msg.twist.twist.linear
        self.state_lin_vel.y *= -1
        self.state_lin_vel.z *= -1

        self.state_ang_vel = msg.twist.twist.angular
        self.state_ang_vel.y *= -1
        self.state_ang_vel.z *= -1
        lock.release()

    def imu_callback(self, msg):
        lock.acquire()
        self.imu = msg #SAVE FRESH IMU
        lock.release()

    def status_callback(self, msg):
        lock.acquire()
        self.status = msg #SAVE FRESH STATUS
        lock.release()

    def land(self):
        msg = Empty()
        self.land_pub.publish(msg)

    def takeoff(self):
        msg = Empty()
        self.takeoff_pub.publish(msg)

    def stop(self):
        """ Reset the robot """
        self.odom_sub.unregister()
        self.imu_sub.unregister()
        self.status_sub.unregister()

    def emergency_stop(self):
        msg = Empty()
        print("Emergency stop")
        self.emergency_pub.publish(msg)

    def send_velocity(self):
        velocity = Twist()
        velocity.linear.x = Vx
        velocity.linear.y = Vy
        velocity.angular.z = Wz
        self.robotino_cmd_vel_pub.publish(velocity)
    #def get_theta(self)
    #    if (self.state_orientation.z > math.pi):
#	    theta = 
#	else:
#	    theta = 

    def stabilization(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        self.robotino_cmd_vel_pub.publish(velocity)

    # def update():
    #     #main loop
    #     while not rospy.is_shutdown():


if __name__ == '__main__':
    rospy.init_node("main_solve_node")
    drone = DroneController()
    PID_Z = PID(0.8, 0, 0, 0.02)
    dt = 0.05
    flag_takeoff = False

    h = 1.0
    dh = 0.5
    theta = 0.5

    state = 0

    r = rospy.Rate(1/dt)
    while not rospy.is_shutdown():
            drone.update()
            try:
                #take off
                if state == 0:
                    if flag_takeoff == False:
                        drone.takeoff()
                        flag_takeoff = True
                        rospy.sleep(4)
                    #make sure state pos z ++
                    if #erorr > 0.05
                        Vz = PID_Z.updatePidControl(h, drone.state_position.z, dt) #z
                        continue



                    
                #change theta
                elif state == 1:
                    PID.updatePidControl() #theta
                #change height
                elif state == 2:
                    PID.updatePidControl(drone.state_position.z, , dt) #z
                #land
                elif state == 3:
                    drone.land()
                state += 1
            except rospy.ROSInterruptException as e:
                drone.emergency_stop()
                drone.stop()
                del drone
                print('End')
            r.sleep()
    drone.takeoff()
    rospy.sleep(10)
    drone.land()
    drone.stop()
    del drone
    print('End')





