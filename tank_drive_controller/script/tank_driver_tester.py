#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tank_interface.msg import MotorRPMArray
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class SimulatedRobot:
    def __init__(self):
        # Robot physical parameters
        self.mass = 20.0  # kg
        self.moment_of_inertia = 2.0  # kg*m^2
        self.wheel_radius = 0.075  # m
        self.wheel_separation = 0.5  # m
        
        # Current state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_accel = 0.0
        self.angular_accel = 0.0
        
        # Motion constraints
        self.max_linear_accel = 1.0  # m/s^2
        self.max_angular_accel = 2.0  # rad/s^2
        self.friction_coeff = 0.1
        self.slip_threshold = 0.8
        
        # Noise parameters
        self.vel_noise_std = 0.05
        self.wheel_noise_std = 2.0
        
        # Delay simulation
        self.command_delay = 0.05
        self.command_history = []
        
    def update(self, target_linear, target_angular, dt):
        # Apply command delay
        self.command_history.append((time.time(), target_linear, target_angular))
        self.command_history = [cmd for cmd in self.command_history 
                              if time.time() - cmd[0] <= self.command_delay]
        
        if self.command_history:
            _, target_linear, target_angular = self.command_history[0]
        
        # Calculate forces and torques
        linear_force = self.mass * (target_linear - self.linear_vel)
        angular_torque = self.moment_of_inertia * (target_angular - self.angular_vel)
        
        # Apply friction
        friction_force = -self.friction_coeff * self.linear_vel
        friction_torque = -self.friction_coeff * self.angular_vel
        
        # Calculate accelerations
        self.linear_accel = (linear_force + friction_force) / self.mass
        self.angular_accel = (angular_torque + friction_torque) / self.moment_of_inertia
        
        # Limit accelerations
        self.linear_accel = np.clip(self.linear_accel, -self.max_linear_accel, self.max_linear_accel)
        self.angular_accel = np.clip(self.angular_accel, -self.max_angular_accel, self.max_angular_accel)
        
        # Update velocities
        self.linear_vel += self.linear_accel * dt
        self.angular_vel += self.angular_accel * dt
        
        # Add noise to velocities
        noisy_linear = self.linear_vel + np.random.normal(0, self.vel_noise_std)
        noisy_angular = self.angular_vel + np.random.normal(0, self.vel_noise_std)
        
        # Calculate wheel speeds with slip simulation
        left_wheel_speed = (noisy_linear - noisy_angular * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_speed = (noisy_linear + noisy_angular * self.wheel_separation / 2) / self.wheel_radius
        
        left_wheel_speed = self.apply_slip(left_wheel_speed)
        right_wheel_speed = self.apply_slip(right_wheel_speed)
        
        # Convert to RPM and add noise
        left_rpm = left_wheel_speed * 60 / (2 * np.pi) + np.random.normal(0, self.wheel_noise_std)
        right_rpm = right_wheel_speed * 60 / (2 * np.pi) + np.random.normal(0, self.wheel_noise_std)
        
        return noisy_linear, noisy_angular, left_rpm, right_rpm
    
    def apply_slip(self, wheel_speed):
        abs_speed = abs(wheel_speed)
        if abs_speed > self.slip_threshold:
            slip_factor = 1.0 - 0.2 * (abs_speed - self.slip_threshold)
            wheel_speed *= max(0.5, slip_factor)
        return wheel_speed

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Create subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.actual_twist_pub = self.create_publisher(Twist, 'actual_twist', 10)
        self.rpm_pub = self.create_publisher(MotorRPMArray, '/actual_wheel_speed', 10)
        
        # Initialize simulated robot
        self.robot = SimulatedRobot()
        self.current_cmd = Twist()
        
        # Data storage for plotting
        self.max_points = 200
        self.start_time = time.time()
        
        # Initialize deques
        self.times = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.target_linear_vel = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.actual_linear_vel = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.target_angular_vel = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.actual_angular_vel = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.fl_rpm = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.fr_rpm = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.rl_rpm = deque([0.0] * self.max_points, maxlen=self.max_points)
        self.rr_rpm = deque([0.0] * self.max_points, maxlen=self.max_points)
        
        # Setup plotting
        self.setup_plotting()
        
        # Create timer for robot simulation (100Hz)
        self.create_timer(0.01, self.update_robot)
        
    def cmd_vel_callback(self, msg):
        self.current_cmd = msg
        self.target_linear_vel.append(msg.linear.x)
        self.target_angular_vel.append(msg.angular.z)
        
    def update_robot(self):
        # Update robot simulation
        linear_vel, angular_vel, left_rpm, right_rpm = self.robot.update(
            self.current_cmd.linear.x,
            self.current_cmd.angular.z,
            0.01
        )
        
        # Store data
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        self.actual_linear_vel.append(linear_vel)
        self.actual_angular_vel.append(angular_vel)
        
        # Publish actual twist
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.actual_twist_pub.publish(twist_msg)
        
        # Publish RPMs
        rpm_msg = MotorRPMArray()
        rpm_msg.rpm = [left_rpm, right_rpm, left_rpm, right_rpm]
        self.rpm_pub.publish(rpm_msg)
        
        # Store RPM data
        self.fl_rpm.append(left_rpm)
        self.fr_rpm.append(right_rpm)
        self.rl_rpm.append(left_rpm)
        self.rr_rpm.append(right_rpm)
        
        # Update plot
        self.update_plot()
        
    def setup_plotting(self):
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Velocity plot
        self.linear_target_line, = self.ax1.plot([], [], 'r--', label='Target Linear')
        self.linear_actual_line, = self.ax1.plot([], [], 'r-', label='Actual Linear')
        self.angular_target_line, = self.ax1.plot([], [], 'b--', label='Target Angular')
        self.angular_actual_line, = self.ax1.plot([], [], 'b-', label='Actual Angular')
        
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Velocity (m/s, rad/s)')
        self.ax1.legend()
        self.ax1.grid(True)
        
        # RPM plot
        self.fl_rpm_line, = self.ax2.plot([], [], 'r-', label='FL RPM')
        self.fr_rpm_line, = self.ax2.plot([], [], 'b-', label='FR RPM')
        self.rl_rpm_line, = self.ax2.plot([], [], 'g-', label='RL RPM')
        self.rr_rpm_line, = self.ax2.plot([], [], 'y-', label='RR RPM')
        
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('RPM')
        self.ax2.legend()
        self.ax2.grid(True)
        
        self.ax1.set_ylim(-1, 1)
        self.ax2.set_ylim(-100, 100)
        
        plt.tight_layout()
        
    def update_plot(self):
        # Convert deques to lists for plotting
        times_list = list(self.times)
        
        # Update lines
        self.linear_target_line.set_data(times_list, list(self.target_linear_vel))
        self.linear_actual_line.set_data(times_list, list(self.actual_linear_vel))
        self.angular_target_line.set_data(times_list, list(self.target_angular_vel))
        self.angular_actual_line.set_data(times_list, list(self.actual_angular_vel))
        
        self.fl_rpm_line.set_data(times_list, list(self.fl_rpm))
        self.fr_rpm_line.set_data(times_list, list(self.fr_rpm))
        self.rl_rpm_line.set_data(times_list, list(self.rl_rpm))
        self.rr_rpm_line.set_data(times_list, list(self.rr_rpm))
        
        # Update axis limits
        if len(times_list) > 1:
            xmin = min(times_list)
            xmax = max(times_list)
            self.ax1.set_xlim(xmin, xmax)
            self.ax2.set_xlim(xmin, xmax)
            
            # Update y-axis limits
            ymin = min(min(self.actual_linear_vel), min(self.actual_angular_vel), -1)
            ymax = max(max(self.actual_linear_vel), max(self.actual_angular_vel), 1)
            self.ax1.set_ylim(ymin - 0.1, ymax + 0.1)
            
            rpm_min = min(min(self.fl_rpm), min(self.fr_rpm), min(self.rl_rpm), min(self.rr_rpm))
            rpm_max = max(max(self.fl_rpm), max(self.fr_rpm), max(self.rl_rpm), max(self.rr_rpm))
            self.ax2.set_ylim(rpm_min - 10, rpm_max + 10)
        
        plt.draw()
        plt.pause(0.01)

def main():
    rclpy.init()
    simulator = RobotSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()