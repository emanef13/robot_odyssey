#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velosity
mode_selection = 4 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear

        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection

        """Computes and publishes wheel velocities based on cmd_vel input."""
        # Compute left and right wheel velocities using differential drive kinematics
        v_l = (vel_msg.linear.x - vel_msg.angular.z * self.wheel_base / 2) / self.wheel_radius
        v_r = (vel_msg.linear.x + vel_msg.angular.z * self.wheel_base / 2) / self.wheel_radius

        # Apply same velocity to front and rear wheels
        self.vel[0] = v_l  # Left front
        self.vel[1] = v_r  # Right front
        self.vel[2] = v_l  # Left rear
        self.vel[3] = v_r  # Right rear

        # Publish wheel velocities
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_vel.publish(vel_array)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection

        vel_msg.linear.x = data.axes[1]*0.3
        vel_msg.angular.z = data.axes[2]

class Teleop_subscriber(Node):

    def __init__(self):
        super().__init__('teleop_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection

        vel_msg.linear.x = data.linear.x
        vel_msg.angular.z = data.angular.z

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()
    teleop_subscriber = Teleop_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(teleop_subscriber)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

