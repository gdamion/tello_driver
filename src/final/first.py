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
from pid import *

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

        #ERRORS
        self.x_error = 0.0
        self.y_error = 0.0
        self.z_error = 0.0
        self.theta_error = 0.0
        self.dist = 0.0

        self.precision = 0.10

    def odom_callback(self, msg):
        lock.acquire()
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.state_position = msg.pose.pose.position
        self.state_position.y *= -1
        self.state_position.z *= -1

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

    def send_velocity(self, Vx, Vy, Vz, Wz):
        velocity = Twist()
        velocity.linear.x = Vx
        velocity.linear.y = -Vy
        velocity.linear.z = -Vz
        velocity.angular.z = -Wz
        self.cmd_vel_pub.publish(velocity)

    #def theta_stabilization(self)
    #    if (self.state_orientation.z > math.pi):
#	    theta = arctan2(y_err, x_err) + 2*math.pi
#	else:
#	    theta = arctan2(y_err, x_err)

    def get_error(self, goal_x, goal_y, goal_z, goal_theta):
        self.x_erorr = goal_x - self.state_position.x
        self.y_erorr = goal_y - self.state_position.y
        self.z_error = goal_z - self.state_position.z
        self.theta_erorr = goal_theta - self.state_orientation.z
        self.dist = sqrt(self.x_error ** 2 + self.y_error ** 2)

    def stabilization(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.z = 0
        self.cmd_vel_pub.publish(velocity)

    # def update():
    #     #main loop
    #     while not rospy.is_shutdown():


if __name__ == '__main__':
    rospy.init_node("main_solve_node")
    drone = DroneController()
    PID_Z = PID(1.15, 0.4, 0.0, 0.5)
    dt = 0.05
    flag_takeoff = False

    h = 1.5
    dh = 0.5
    theta = 150 * pi / 180
    theta += drone.state_orientation.z

    state = 0
    fix_ang = 0

    r = rospy.Rate(1/dt)
    while not rospy.is_shutdown():
        r.sleep()
        print("Err: " + str(round(drone.x_error, 4)) + " " + str(round(drone.y_error, 4)) + " " + str(round(drone.z_error, 4)) + " " + str(round(drone.theta_error, 4)))

        try:
            #take off
            if state == 0:
                drone.get_error(0.0, 0.0, h, theta - fix_ang)
                if flag_takeoff == False:
                    drone.takeoff()
                    print("Takeoff")
                    flag_takeoff = True
                #make sure state pos z ++
                if abs(drone.z_error) > drone.precision:
                    Vz = PID_Z.updatePidControl(h, drone.state_position.z, dt) #z
                    print("Control V: " + str(Vz))
                    drone.send_velocity(0.0, 0.0, Vz, 0.0)
                    continue
                else:
                    print("THE GOAL 0 IS REACHED")
                    drone.stabilization()
                    print("SLEEPING")
                    rospy.sleep(2)
                    print("STOP SLEEPING")
                    # drone.land()
                    fix_ang = drone.state_orientation.z
                    state = 1

            #change theta
            if state == 1:
                drone.get_error(0.0, 0.0, h, theta - fix_ang)
                Kr = 0.7
                theta_err = drone.theta_erorr
                theta_err += 2*math.pi if drone.state_orientation.z > math.pi else 0

                print("Theta err: " + str(theta_err) + " Theta: " + str(drone.state_orientation.z) + " Fix_angle: " + str(fix_ang))

                if abs(theta_err) > 10 * pi / 180:
                    drone.send_velocity(0.0, 0.0, 0.0, Kr * theta_err)
                    continue
                else:
                    print("THE GOAL 1 IS REACHED")
                    drone.stabilization()
                    print("SLEEPING")
                    rospy.sleep(2)
                    print("STOP SLEEPING")
                    drone.land()
                    state = 2

            # #change height
            # elif state == 2:
            #     PID.updatePidControl(drone.state_position.z, , dt) #z
            # #land
            # elif state == 3:
            #     drone.land()
        except rospy.ROSInterruptException as e:
            drone.emergency_stop()
            drone.stop()
            del drone
            print('End')

    # drone.takeoff()
    # rospy.sleep(10)
    # drone.land()
    drone.stop()
    del drone
    print('End')





