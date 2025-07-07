#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Float64MultiArray, Float64
from geometry_msgs.msg import Pose

import numpy as np

class SyncCoordinator(Node):
    def __init__(self):
        super().__init__('sync_coordinator')

        # Publicadores
        self.uav_speed_xy_pub = self.create_publisher(Float32, '/uav/reference_speed_xy', 10)
        self.uav_speed_z_pub = self.create_publisher(Float32, '/uav/reference_speed_z', 10)
        self.ugv_speed_pub = self.create_publisher(Float32, '/ugv/reference_speed', 10)

        # Subscriptores
        self.create_subscription(Int32, '/uav/waypoint_index', self.uav_wp_callback, 10)
        self.create_subscription(Int32, '/ugv/waypoint_index', self.ugv_wp_callback, 10)
        self.create_subscription(Int32, '/tether/waypoint_index', self.tether_wp_callback, 10)
        self.target_length_suscriber = self.create_subscription(Float64MultiArray, '/cable_length', self.target_cable_length_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, '/rs_robot/ugv_gt_pose', self.ugv_pose_callback, 10)
        self.uav_pose_subscriber = self.create_subscription(Pose, 'sjtu_drone/gt_pose', self.uav_pose_callback, 10)

        # Parámetros de sincronización
        self.nominal_uav_speed = 0.5  # m/s
        self.nominal_ugv_speed = 0.5  # m/s
        self.max_overshoot = 1        # número máximo de waypoints de ventaja o desventaja permitida
        self.min_speed = 0.2
        self.max_speed_uav = 1.0
        self.max_speed_ugv = 4.0

        # Inicializar índices
        self.uav_wp_index = 0
        self.ugv_wp_index = 0
        self.tether_wp_index = 0
        self.cable_length = 0.6  
        self.target_cable_length = 0.0
        self.distance = 0.0

        # Temporizador
        timer_period = 0.05
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.compute_speeds)

    def uav_wp_callback(self, msg):
        self.uav_wp_index = msg.data

    def tether_wp_callback(self, msg):
        self.tether_wp_index = msg.data

    def ugv_wp_callback(self, msg):
        self.ugv_wp_index = msg.data

    def target_cable_length_callback(self, msg):
        if len(msg.data) >= 2:
            self.cable_length = float(msg.data[0])
            self.target_cable_length = float(msg.data[1])
        else:
            self.get_logger().warn("Mensaje de cable_lengthPUB recibido con menos de 2 elementos.")

    def ugv_pose_callback(self, msg):
        self.ugv_position = msg

    def uav_pose_callback(self, msg):
        self.uav_position = msg

    def compute_speeds(self):
        # Diferencia entre índices
        delta = self.uav_wp_index - self.ugv_wp_index

        # Inicializar con velocidades nominales
        uav_speed = self.nominal_uav_speed
        ugv_speed = self.nominal_ugv_speed

        # Si el UAV va muy adelantado
        if delta > self.max_overshoot:
            self.get_logger().warn("UAV va demasiado adelantado, frenando UAV y acelerando UGV.")
            uav_speed *= 0.1  # frena el UAV
            ugv_speed *= 3.0  # acelera el UGV

        # Si el UGV va demasiado adelantado
        elif delta < -self.max_overshoot:
            self.get_logger().warn("UGV va demasiado adelantado, frenando UGV y acelerando UAV.")
            uav_speed *= 1.5  # acelera el UAV
            ugv_speed *= 0.1  # frena el UGV

        # Saturación de velocidades
        uav_speed = min(max(uav_speed, self.min_speed), self.max_speed_uav)
        ugv_speed = min(max(ugv_speed, self.min_speed), self.max_speed_ugv)

        # Publicar
        self.uav_speed_xy_pub.publish(Float32(data=uav_speed))
        self.uav_speed_z_pub.publish(Float32(data=uav_speed))
        self.ugv_speed_pub.publish(Float32(data=ugv_speed))
 
        # Log
        l_error = self.target_cable_length - self.cable_length

        self.get_logger().info(f"UAV - UGV")
        self.get_logger().info(f"Velocidades publicadas UAV: {uav_speed:.2f} m/s, UGV: {ugv_speed:.2f} m/s")
        self.get_logger().info(f"delta: {delta}")
        self.get_logger().info(f"wp_uav: {self.uav_wp_index} , wp_ugv: {self.ugv_wp_index}")
        self.get_logger().info(f"")
        self.get_logger().info(f"TETHER")
        self.get_logger().info(f"delta: {l_error}")
        self.get_logger().info(f"Cable actual: {self.cable_length:.2f}, objetivo: {self.target_cable_length:.2f}")
        self.get_logger().info(f"wp_tether: {self.tether_wp_index}")
        self.get_logger().info(f"-----------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = SyncCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
