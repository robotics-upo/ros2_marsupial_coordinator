#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float64
import time
from std_msgs.msg import Float32
from rclpy.clock import Clock

class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        self.target_position = Pose()
        self.current_position = Pose()
        self.uav_position = Pose()
        self.target_cable_length = 0.0

        self.cable_length = 0.6    
        self.safety_margin = 0.5
        self.tether_coef = 1.1

        self.kp_winch = 3.0
        self.ki_winch = 0.05
        self.kd_winch = 0.5

        # self.kp_winch = 10.0
        # self.ki_winch = 0.1
        # self.kd_winch = 1.0

        self.velocity_winch_limit = 2.0
        self.winch_position_x = -0.25
        self.winch_position_z = 0.35

        self.integral_error = 0.0
        self.previous_error = 0.0

        timer_period = 0.02

        # self.constant_speed = 1
        self.tolerance = 0.1

        self.pos = np.array([0, 0, 0, 0], float)
        self.vel = np.array([0, 0, 0, 0, 0], float)
 
        self.last_time = None

        # Posición actual del UGV
        self.pose_subscriber = self.create_subscription(Pose, '/rs_robot/ugv_gt_pose', self.ugv_pose_callback, 10)

        # Posición actual del UAV
        self.uav_pose_subscriber = self.create_subscription(Pose, 'sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        
        # Posición de referencia del UGV
        self.target_subscriber = self.create_subscription(Pose, '/ugv/reference_pose', self.ugv_reference_callback, 10)

        # Comandos del controlador para el UGV
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # Publicador para la longitud de la cuerda y la longitud objetivo
        self.pub_cable_length = self.create_publisher(Float64MultiArray, '/cable_lengthPUB', 10)

        # Velocidad de referencia del UGV
        self.speed_subscriber = self.create_subscription(Float32, '/ugv/reference_speed', self.speed_callback, 10)

        # Suscripción al target_length
        self.target_length_suscriber = self.create_subscription(Float64, '/cable_length', self.target_cable_length_callback, 10)

        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('UGV controller has been started.')

    def target_cable_length_callback(self, msg):
        self.target_cable_length = msg.data

    def speed_callback(self, msg):
        self.desired_speed = msg.data

    def ugv_pose_callback(self, msg):
        self.current_position = msg

    def uav_pose_callback(self, msg):
        self.uav_position = msg

    def ugv_reference_callback(self, msg):
        self.target_position = msg

    def calculate_winch_velocity(self):
        length_error = (self.target_cable_length + self.safety_margin) - self.cable_length
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = current_time
            self.previous_winch_velocity = 0.0  # Inicializamos la velocidad anterior
            return 0.0, length_error

        dt = current_time - self.last_time

        # Evitar pasos con dt muy grandes o negativos (ej. al reanudar simulación)
        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warn(f'[SKIP] dt anómalo: {dt:.3f}, now: {current_time:.3f}, last: {self.last_time:.3f}')
            self.last_time = current_time
            return 0.0, length_error

        self.integral_error += length_error * dt

        raw_derivative = (length_error - self.previous_error) / dt

        winch_velocity = (
            self.kp_winch * length_error +
            self.ki_winch * self.integral_error +
            self.kd_winch * raw_derivative
        )

        winch_velocity = max(min(winch_velocity, self.velocity_winch_limit), -self.velocity_winch_limit)

        self.cable_length += winch_velocity * dt
        self.previous_error = length_error
        self.previous_winch_velocity = winch_velocity
        self.last_time = current_time

        return winch_velocity, length_error


    
    def control_loop(self):
        if self.current_position is None or self.uav_position is None or self.target_position is None:
            return

        direction_x = self.target_position.position.x - self.current_position.position.x
        direction_y = self.target_position.position.y - self.current_position.position.y

        magnitude = math.sqrt(direction_x**2 + direction_y**2)

        vel_msg = Float64MultiArray()
        pos_msg = Float64MultiArray()
        cable_length_msg = Float64MultiArray()

        if magnitude < self.tolerance:
            control_xy = 0.0
        else:
            sign = np.sign(direction_x)
            control_xy = self.desired_speed * sign

        if direction_x != 0 and magnitude > self.tolerance:
            steering_angle = math.atan(direction_y/direction_x)
        else:
            steering_angle = 0.0

        winch_velocity, length_error = self.calculate_winch_velocity()

        pos_msg.data = [float(steering_angle), float(steering_angle), float(steering_angle), float(steering_angle)]
        vel_msg.data = [float(control_xy), float(control_xy), float(control_xy), float(control_xy), float(winch_velocity)]
        cable_length_msg.data = [float(self.cable_length), float(self.target_cable_length)]

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        self.pub_cable_length.publish(cable_length_msg)

        #self.get_logger().info(f'Winch velocity: {winch_velocity:.3f}')
        
        self.get_logger().info(f'Target: {self.target_cable_length+self.safety_margin:.3f}, Length: L={self.cable_length:.3f}, Error={length_error:.3f}, Winch velocity: {winch_velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    ugv_controller = UGVController()
    rclpy.spin(ugv_controller)
    ugv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
