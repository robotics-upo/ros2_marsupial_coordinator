#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
from std_msgs.msg import Empty, Float32, Int32

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self.target_position = Pose()
        self.current_position = None
        self.desired_speed_xy = 0.0
        self.desired_speed_z = 0.0

        timer_period = 0.02 # 50 Hz

        self.pose_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.pose_callback, 10)
        self.target_subscriber = self.create_subscription(Pose, '/uav/reference_pose', self.target_callback, 10)
        self.speed_xy_subscriber = self.create_subscription(Float32, '/uav/reference_speed_xy', self.speed_xy_callback, 10)
        self.speed_z_subscriber = self.create_subscription(Float32, '/uav/reference_speed_z', self.speed_z_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, '/sjtu_drone/cmd_vel', 10)
        self.pub_uav_arrived = self.create_publisher(Int32, '/uav/arrived', 10)

        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('Drone controller has been started.')

    def pose_callback(self, msg):
        self.current_position = msg.position
 
    def target_callback(self, msg):
        self.target_position = msg

    def speed_xy_callback(self, msg):
        self.desired_speed_xy = msg.data

    def speed_z_callback(self, msg):
        self.desired_speed_z = msg.data

    def control_loop(self):

        if self.current_position is None or self.target_position is None:
            return

        direction_x = self.target_position.position.x - self.current_position.x
        direction_y = self.target_position.position.y - self.current_position.y
        direction_z = self.target_position.position.z - self.current_position.z

        distance_xy = math.hypot(direction_x, direction_y)
        
        cmd_vel = Twist()

        control_x = (direction_x / distance_xy) * self.desired_speed_xy
        control_y = (direction_y / distance_xy) * self.desired_speed_xy
        control_z = (direction_z / abs(direction_z)) * self.desired_speed_z

        cmd_vel.linear.x = control_x
        cmd_vel.linear.y = control_y
        cmd_vel.linear.z = control_z


        self.velocity_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ugv_controller = DroneController()
    rclpy.spin(ugv_controller)
    ugv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()