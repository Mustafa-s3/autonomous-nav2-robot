#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # 1. Hardware Pins Setup
        # Motors
        self.left_in3 = DigitalOutputDevice(17)
        self.left_in4 = DigitalOutputDevice(18)
        self.left_pwm = PWMOutputDevice(13, frequency=100)
        self.right_in1 = DigitalOutputDevice(22)
        self.right_in2 = DigitalOutputDevice(23)
        self.right_pwm = PWMOutputDevice(12, frequency=100)
        
        # Quadrature Encoders (A and B phases)
        # bounce_time=0.0001 (0.1ms) helps filter electrical noise
        self.left_encoder_a = DigitalInputDevice(5, pull_up=True, bounce_time=0.0001)
        self.left_encoder_b = DigitalInputDevice(6, pull_up=True, bounce_time=0.0001)
        self.right_encoder_a = DigitalInputDevice(26, pull_up=True, bounce_time=0.0001)
        self.right_encoder_b = DigitalInputDevice(27, pull_up=True, bounce_time=0.0001)

        # 2. Robot Parameters
        self.wheel_radius = 0.0325      # meters
        self.wheel_base = 0.22          # meters
        self.ticks_per_rev = 822        # Adjust based on manual rotation test
        self.wheel_circumference = 2 * math.pi * self.wheel_radius
        
        # 3. Odometry State
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 4. ROS Interfaces
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Interrupts (Only assigned ONCE to avoid double-counting)
        self.left_encoder_a.when_activated = self.left_encoder_callback
        self.right_encoder_a.when_activated = self.right_encoder_callback

        # Timer for odometry calculation (50 Hz)
        self.timer = self.create_timer(0.02, self.odometry_timer_callback)
        
        self.get_logger().info("Motor controller with Quadrature Odometry started")

    # --- Encoder Callbacks ---
    def left_encoder_callback(self):
        # Quadrature logic: compare Phase A and B to determine direction
        if self.left_encoder_a.value != self.left_encoder_b.value:
            self.left_ticks += 1
        else:
            self.left_ticks -= 1

    def right_encoder_callback(self):
        # NOTE: If your right wheel drives the opposite way in RViz, 
        # change "!=" to "==" in the line below.
        if self.right_encoder_a.value != self.right_encoder_b.value:
            self.right_ticks -= 1
        else:
            self.right_ticks += 1

    # --- Odometry Math ---
    def odometry_timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001: return
        
        # 1. Calculate wheel displacements
        d_left_ticks = self.left_ticks - self.prev_left_ticks
        d_right_ticks = self.right_ticks - self.prev_right_ticks
        
        left_dist = (d_left_ticks / self.ticks_per_rev) * self.wheel_circumference
        right_dist = (d_right_ticks / self.ticks_per_rev) * self.wheel_circumference
        
        # 2. Update Pose (Differential Drive Model)
        d_dist = (left_dist + right_dist) / 2.0
        d_theta = (right_dist - left_dist) / self.wheel_base
        
        if abs(d_theta) < 1e-6: # Moving straight
            self.x += d_dist * math.cos(self.theta)
            self.y += d_dist * math.sin(self.theta)
        else: # Arc movement (ICC)
            radius = (self.wheel_base / 2.0) * (left_dist + right_dist) / (right_dist - left_dist)
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + d_theta) - math.cos(self.theta))
            self.theta += d_theta

        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 3. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        
        # Velocity estimation
        odom.twist.twist.linear.x = d_dist / dt
        odom.twist.twist.angular.z = d_theta / dt
        self.odom_pub.publish(odom)

        # 4. Publish Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        # 5. Publish Joint States (for spinning wheels in RViz)
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [
            (self.left_ticks / self.ticks_per_rev) * 2 * math.pi,
            (self.right_ticks / self.ticks_per_rev) * 2 * math.pi
        ]
        self.joint_pub.publish(js)

        # Update previous values
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.last_time = current_time

    # --- Motor Control ---
    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        left_speed = v - (w * self.wheel_base / 2)
        right_speed = v + (w * self.wheel_base / 2)
        
        # Map speed (m/s) to duty cycle (-1.0 to 1.0)
        # Adjust max_speed based on your motor capabilities
        max_speed = 0.5 
        self.set_motor(self.left_in3, self.left_in4, self.left_pwm, left_speed / max_speed)
        self.set_motor(self.right_in1, self.right_in2, self.right_pwm, right_speed / max_speed)

    def set_motor(self, pin_a, pin_b, pwm, duty):
        duty = max(-1.0, min(1.0, duty)) # Clamp
        if duty > 0:
            pin_a.on(); pin_b.off()
        elif duty < 0:
            pin_a.off(); pin_b.on()
        else:
            pin_a.off(); pin_b.off()
        pwm.value = abs(duty)

    def destroy_node(self):
        self.left_pwm.off()
        self.right_pwm.off()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()