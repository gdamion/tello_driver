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
from trajectory_generator import *

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
        # self.fast_mode_pub = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1, latch=True)
        # self.flattrim_pub = rospy.Publisher('/tello/flattrim', Empty, queue_size=1, latch=True)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1, latch=True)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1, latch=True)
        # self.manual_takeoff_pub = rospy.Publisher('/tello/manual_takeoff', Empty, queue_size=1, latch=True)

        #ROS SUB
        self.odom_sub = rospy.Subscriber('/tello/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/tello/imu', Imu, self.imu_callback)
        self.status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.status_callback)

        self.trajectory_sub = rospy.Subscriber('/tello/trajectory', CartesianTrajectory, self.trajectory_callback)

        #DRONE STATE
        self.imu = Imu()
        self.status = TelloStatus()

        self.state_position = Point()
        self.state_orientation = Vector3()
        self.state_lin_vel = Vector3()
        self.state_ang_vel = Vector3()

        self.start_time = 0.0
        self.cart_trajectory = CartesianTrajectory()
        self.i = 0
        self.precision = 0.1

        #ERRORS
        self.x_error = 0.0
        self.y_error = 0.0
        self.z_error = 0.0
        self.theta_error = 0.0
        self.dist = 0.0
        self.x_offset = 0.0
        self.y_offset = 0.0

    def trajectory_callback(self, msg):
        lock.acquire()
        self.cart_trajectory = msg
        lock.release()

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
        velocity.angular.z = Wz
        self.cmd_vel_pub.publish(velocity)

    # def theta_stabilization(self)
    #    if (self.state_orientation.z > math.pi):
	#     theta = arctan2(y_err, x_err) + 2*math.pi
	# else:
	#     theta = arctan2(y_err, x_err)

    def get_error(self, goal_x, goal_y, goal_z, goal_theta):
        self.x_error = goal_x - self.state_position.x
        self.y_error = goal_y - self.state_position.y
        self.z_error = goal_z - self.state_position.z
        self.theta_error = goal_theta - self.state_orientation.z
        self.dist = sqrt(self.x_error ** 2 + self.y_error ** 2)
        s = "Time: " + str(time.time()) + " | X_err: " + self.x_error + " | Y_err: "
        + self.y_error + " \nZ_err: " + self.z_error + " | Theta_err" + self.theta_error
        # print(s)

    def stabilization(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.z = 0
        self.cmd_vel_pub.publish(velocity)

    def check_distance(self, iter, pose_x, pose_y):
        for i in range(iter, len(self.cart_trajectory.poses)):
            x_error = self.cart_trajectory.poses[i].x - pose_x
            y_error = self.cart_trajectory.poses[i].y - pose_y
            distance = sqrt(x_error**2 + y_error**2)
            if (distance >= 0.1):
                return i

#Замечания
# Угол желаемый считать относительно текущей и следующей точки
# Нужен контроллер угла
# Исправить, чтобы мы ехали строго к ближайшей точке, если d > d_err, а не перескакивали на следующие в массиве
# Увеличиит скорость, подобрав коэффы пида
# Попровать подключить фильтр Калмана
# Следить за точностью отработки, тк визуальная одометрия плавает
# В 1 и 2 заданиях всегда считать ошибки по всем координатам, чтобы публиковать актуальную инфу в файл

if __name__ == '__main__':
    rospy.init_node("3rd_task_solve_node")
    drone = DroneController()

    PID_X = PID(0.8, 0.1, 0.0, 0.5)
    PID_Y = PID(0.8, 0.1, 0.0, 0.5)
    PID_Z = PID(1.15, 0.4, 0.0, 0.5)

    ### INPUT YOUR PARAMETERS HERE
    h = 0.3
    ###

    Kr = 0.7
    flag_takeoff = False
    theta = 90 * pi / 180
    state = 0
    fix_ang = 0
    dt = 0.05
    flight_duration = 300
    r = rospy.Rate(1/dt)

    while not rospy.is_shutdown():
        r.sleep()
        drone.get_error(0.0, 0.0, h, theta - fix_ang)
        try:

            # TAKEOFF AND REACH THE H
            if state == 0:
                if flag_takeoff == False:
                    drone.takeoff()
                    print("TAKEOFF")
                    flag_takeoff = True

                if abs(drone.z_error) > drone.precision: # IF WE HAVEN'T REACHED THE GOAL POINT - MOVE
                    Vz = PID_Z.updatePidControl(h, drone.state_position.z, dt) # CONTROL Z-SPEED
                    drone.send_velocity(0.0, 0.0, Vz, 0.0)
                    continue

                else: # WE'VE REACHED THE GOAL
                    print("THE GOAL 0 IS REACHED: TAKEOFF AND HEIGHT H")
                    drone.stabilization()
                    print("SLEEPING")
                    rospy.sleep(2)
                    print("STOP SLEEPING")
                    fix_ang = drone.state_orientation.z
                    drone.x_offset = drone.state_position.x
                    drone.y_offset = drone.state_position.y
                    state = 1

            #MOVE ALONG 8-LIKE TRAJECTORY
            if state == 1:
                drone.start_time = time.time()

                N = len(drone.cart_trajectory.poses)
                print("N: " + str(N))

                while ((time.time() - drone.start_time) <= flight_duration):

                    if drone.i < N - 2: # If WE ARE NOT OUT OF ARRAY BORDERS
                        goal_pose = drone.cart_trajectory.poses[drone.i]
                        drone.get_error(goal_pose.x, goal_pose.y, 0, goal_pose.theta)
                        iter = drone.check_distance(drone.i, drone.state_position.x, drone.state_position.y)

                        if drone.dist > drone.precision:
                            x_real = goal_pose.x + drone.x_offset
                            y_real = goal_pose.y + drone.y_offset

                            Vx = PID_X.updatePidControl(x_real, drone.state_position.x, dt) #CONTROL X-SPEED
                            Vy = PID_Y.updatePidControl(y_real, drone.state_position.y, dt) #CONTROL Y-SPEED

                            theta = drone.state_orientation.z # YAW ANGLE
                            error_angle = goal_pose.theta - theta # ANGLE BETWEEN CURRENT STATE AND GOAL

                            ### ??? SOS SOS SOS
                            if error_angle >= math.pi:
                                error_angle -= 2*math.pi
                            else:
                                error_angle += 2*math.pi

                            error_angle *= -Kr
                            ### ???

                            print("Vx " + str(Vx) + " Vy " + str(Vy) + " Err_theta" + str(error_angle))

                            drone.send_velocity(Vx, Vy, 0.0, error_angle)
                            drone.i = iter + 1
                            print("iter " + str(iter))
                    else :
                        drone.i = 0
            else:
                print("FLIGHT TIME IS ENDED")
                drone.stabilization()
                print("SLEEPING")
                rospy.sleep(2)
                print("STOP SLEEPING")
                drone.land()
                print("LANDING")
                break

        except rospy.ROSInterruptException as e:
            drone.emergency_stop()
            drone.land()

    drone.stop()
    del drone
    print('End')

