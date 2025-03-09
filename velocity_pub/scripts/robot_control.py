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
        timer_period = 0.02 # 50 Hz control loop
        self.wheel_base = 0.156 # Distance between left and right wheels (meters)
        self.wheel_radius = 0.026 # Wheel radius (meters)

        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear

        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def listener_callback(self, data):
        """Receives velocity commands from cmd_vel and updates linear/angular velocities."""
        self.linear_velocity = data.linear.x
        self.angular_velocity = data.angular.z

    def timer_callback(self):
        """Computes and publishes wheel velocities based on cmd_vel input."""
        v_l = (self.linear_velocity - self.angular_velocity * self.wheel_base / 2) / self.wheel_radius
        v_r = (self.linear_velocity + self.angular_velocity * self.wheel_base / 2) / self.wheel_radius

        self.vel[0] = v_l  # Left front
        self.vel[1] = v_r  # Right front
        self.vel[2] = v_l  # Left rear
        self.vel[3] = v_r  # Right rear

        vel_array = Float64MultiArray(data=self.vel)
        self.pub_vel.publish(vel_array)

class TeleopSubscriber(Node):
    def __init__(self):
        super().__init__('teleop_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        """Passes cmd_vel messages to the robot."""
        pass  # The `Commander` node already listens to `cmd_vel`

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')

        # Joystick configuration
        self.max_linear_speed = 0.3  # Maximum linear speed (m/s)
        self.max_angular_speed = 1.5  # Maximum angular speed (rad/s)

        # Define which axes/buttons control movement
        self.linear_axis = 1  # Left stick vertical (up/down)
        self.angular_axis = 2  # Right stick horizontal (left/right)

        # Create publisher for cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

    def listener_callback(self, data):
        """Convert joystick input to Twist and publish to cmd_vel."""
        twist_msg = Twist()
        
        # Map joystick axes to linear and angular velocities
        twist_msg.linear.x = data.axes[self.linear_axis] * self.max_linear_speed
        twist_msg.angular.z = data.axes[self.angular_axis] * self.max_angular_speed

        self.pub_cmd_vel.publish(twist_msg)

if __name__ == '__main__':
    rclpy.init()

    commander = Commander()
    joy_subscriber = JoySubscriber()
    teleop_subscriber = TeleopSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(teleop_subscriber)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

