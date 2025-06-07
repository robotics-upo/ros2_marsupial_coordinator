#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Pose, '/rs_robot/ugv_gt_pose', self.pose_ugv_callback, 10)
        self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.pose_uav_callback, 10)

    def pose_ugv_callback(self, msg):
        # TF: world -> base_footprint (dinámico) 
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'base_footprint'
        t1.transform.translation.x = msg.position.x
        t1.transform.translation.y = msg.position.y
        t1.transform.translation.z = msg.position.z
        t1.transform.rotation.x = msg.orientation.x
        t1.transform.rotation.y = msg.orientation.y
        t1.transform.rotation.z = msg.orientation.z
        t1.transform.rotation.w = msg.orientation.w

        self.br.sendTransform(t1)

    def pose_uav_callback(self, msg):
        # TF: world -> uav_base_link (dinámico)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'uav_base_link'
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = DynamicTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
