#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float64, Float32
from rclpy.clock import Clock, ClockType

class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        # === CABLE ===
        # Longitud del cable (inicial)
        self.tether_length = 0.5

        # Referencia de longitud de cable recibida del Pure Pursuit
        self.tether_ref_length = 0.5

        # Referencia de longitud a la entrada el PID
        self.target_length = 0.0

        # Margen de seguridad
        self.safety_margin = 0.3
        
        # === UAV ===
        self.uav_position = None
        self.target_uav_position = None
        
        # === WINCH ===
        # Offset
        self.winch_position_x = -0.5
        self.winch_position_z = 0.9

        # Distancia inicial entre winch y UAV y radio del winch
        self.distance = 0.6
        self.radius = 0.1494115                  
        self.effective_radius = 0.1494115   

        # Velocidad inicial
        self.winch_velocity = 0.0

        # Parámetros del PID
        self.kp_winch = 10.0
        self.ki_winch = 0.2
        self.kd_winch = 0.5
 
        self.integral_error = 0.0
        self.previous_error = 0.0     
   
        # === UGV ===
        # Mensaje de sus posiciones
        self.target_position = Pose()
        self.current_position = Pose()

        # Velocidad inicial
        self.desired_speed = 0.0   

        # Factor de ajuste para el actuador
        self.k = 5.74  

        # Tolerancia
        self.tolerance = 0.1

        # === BUCLE DE CONTROL ===
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # === SUSCRIPCIONES ===
        self.pose_subscriber = self.create_subscription(Pose, '/rs_robot/ugv_gt_pose', self.pose_callback, 10)
        self.uav_pose_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        self.speed_subscriber = self.create_subscription(Float32, '/ugv/reference_speed', self.speed_callback, 10)
        self.target_subscriber = self.create_subscription(Pose, '/ugv/reference_pose', self.target_callback, 10)
        self.target_uav_subscriber = self.create_subscription(Pose, '/uav/reference_pose', self.target_uav_callback, 10)
        self.target_length_subscriber = self.create_subscription(Float64, '/tether/reference_length', self.target_length_callback, 10)

        # === PUBLICADORES ===
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_cable_length = self.create_publisher(Float64MultiArray, '/cable_length', 10)

        self.get_logger().info('Controlador del UGV y cabrestante iniciado.')

    def speed_callback(self, msg):
        self.desired_speed = msg.data

    def pose_callback(self, msg):
        self.current_position = msg

    def uav_pose_callback(self, msg):
        self.uav_position = msg

    def target_callback(self, msg):
        self.target_position = msg

    def target_uav_callback(self, msg):
        self.target_uav_position = msg

    def target_length_callback(self, msg):
        self.tether_ref_length = msg.data

    def calculate_winch_velocity(self):
        # Distancia entre el winch y el UAV
        self.distance = math.sqrt(
            (self.uav_position.position.x - (self.current_position.position.x + self.winch_position_x)) ** 2 +
            (self.uav_position.position.y - self.current_position.position.y) ** 2 +
            (self.uav_position.position.z - (self.current_position.position.z + self.winch_position_z)) ** 2
        )

        # Referencia del PID como la recibida del Pure Pursuit + Margen de seguridad
        self.target_length = self.tether_ref_length + self.safety_margin  

        # Error de longitud
        length_error = self.target_length - self.tether_length

        # Salto de tiempo
        self.dt = self.timer_period

        self.integral_error += length_error * self.dt
        self.previous_error = length_error
        derivative_error = (length_error - self.previous_error) / self.dt if self.dt > 0 else 0.0

        # Velocidad lineal
        winch_velocity_linear = (self.kp_winch * length_error + self.ki_winch * self.integral_error + self.kd_winch * derivative_error)

        # Saturación de la velocidad lineal
        max_winch_velocity_linear = 0.4482345
        winch_velocity_linear = max(min(winch_velocity_linear, max_winch_velocity_linear), -max_winch_velocity_linear)

        # Velocidad angular
        winch_velocity_angular = winch_velocity_linear / self.radius

        # Saturación de velocidad angular
        max_winch_velocity_angular = 3.0
        winch_velocity_angular = max(min(winch_velocity_angular, max_winch_velocity_angular), -max_winch_velocity_angular)

        # Estimación de la longitud del cable para la próxima iteración
        self.tether_length += winch_velocity_angular * self.effective_radius * self.dt

        self.get_logger().info(f'Vel linear: {winch_velocity_linear:.5f}, vel angular {winch_velocity_angular:.5f}, dt: {self.dt:.5f}')

        return winch_velocity_angular, length_error
    
    def control_loop(self):
        if self.current_position is None or self.uav_position is None or self.target_position is None:
            return        

        # === CONTROL DEL UGV ===
        direction_x = self.target_position.position.x - self.current_position.position.x
        direction_y = self.target_position.position.y - self.current_position.position.y

        magnitude = math.sqrt(direction_x**2 + direction_y**2)

        vel_msg = Float64MultiArray()
        pos_msg = Float64MultiArray()
        cable_length_msg = Float64MultiArray()

        # La tolerancia es inferior a la lookahead distance del Pure Pursuit, por lo que se detiene en el último punto
        if magnitude < self.tolerance:
            control_xy = 0.0 
        else:
            sign = np.sign(direction_x)
            control_xy = (self.desired_speed*self.k) * sign

        if direction_x != 0 and magnitude > self.tolerance:
            steering_angle = math.atan(direction_y/direction_x)
        else:
            steering_angle = 0.0

        # === CONTROL DEL CABRESTANTE ===
        self.winch_velocity, length_error = self.calculate_winch_velocity()

        # === PUBLICACIÓN ===
        pos_msg.data = [float(steering_angle), float(steering_angle), float(steering_angle), float(steering_angle)]
        vel_msg.data = [float(control_xy), float(control_xy), float(control_xy), float(control_xy), float((self.winch_velocity)*(self.k*self.effective_radius))]
        cable_length_msg.data = [float(self.tether_length), float(self.target_length), float(self.distance)]

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        self.pub_cable_length.publish(cable_length_msg)

        # === DEBUG ===
        self.get_logger().info(f"------CONTROL CABLE------")
        self.get_logger().info(f"Longitud REFERENCIA: {self.tether_ref_length:.3f} m")
        self.get_logger().info(f"Referencia Generada: {self.target_length:.3f} m")
        self.get_logger().info(f"L estimada: {self.tether_length:.3f} m | Error (RG-LE)={length_error:.3f}")
        self.get_logger().info(f"Velocidad del cabrestante: {self.winch_velocity:.3f} angular")

def main(args=None):
    rclpy.init(args=args)
    ugv_controller = UGVController()
    rclpy.spin(ugv_controller)
    ugv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
