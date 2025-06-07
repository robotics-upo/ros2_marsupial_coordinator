#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
from std_msgs.msg import Empty
from std_msgs.msg import Float32

 

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self.target_position = Pose()
        self.current_position = None
        
        self.max_speed = 0.2
        self.acceleration = 0.2 
        self.deceleration_distance = 0.5
        self.tolerance = 0.1  
        timer_period = 0.02

        self.takeoff_pub = self.create_publisher(Empty, '/sjtu_drone/takeoff', 10)

        # Posición actual del UAV
        self.pose_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.pose_callback, 10)

        # Posición de referencia del UAV
        self.target_subscriber = self.create_subscription(Pose, '/uav/reference_pose', self.target_callback, 10)
        
        # Comandos de velocidad para el UAV
        self.velocity_publisher = self.create_publisher(Twist, '/sjtu_drone/cmd_vel', 10)

        # Velocidad de referencia para el UAV
        self.speed_subscriber = self.create_subscription(Float32, '/uav/reference_speed', self.speed_callback, 10)
        
        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('Drone controller has been started.')

    def pose_callback(self, msg):
        self.current_position = msg.position

    def target_callback(self, msg):
        self.target_position = msg

    def speed_callback(self, msg):
        self.desired_speed = msg.data

    def control_loop(self):
        if self.current_position is None or self.target_position is None:
            return
        
        direction_x = self.target_position.position.x - self.current_position.x
        direction_y = self.target_position.position.y - self.current_position.y
        direction_z = self.target_position.position.z - self.current_position.z
        
        distance_to_target = math.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
        
        if distance_to_target < self.tolerance: 
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
        else:
            
            
            norm = math.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            control_x = (direction_x / norm) * self.desired_speed
            control_y = (direction_y / norm) * self.desired_speed
            control_z = (direction_z / norm) * self.desired_speed

            cmd_vel = Twist()
            cmd_vel.linear.x = control_x
            cmd_vel.linear.y = control_y
            cmd_vel.linear.z = control_z

        self.velocity_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
