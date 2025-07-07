#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.ugv_pose = None
        self.uav_pose = None

        self.create_subscription(Pose, '/rs_robot/ugv_gt_pose', self.pose_ugv_callback, 10)
        self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.pose_uav_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_transforms)

    def pose_ugv_callback(self, msg):
        self.ugv_pose = msg

    def pose_uav_callback(self, msg):
        self.uav_pose = msg

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        if self.ugv_pose is not None:
            t1 = TransformStamped()
            t1.header.stamp = now
            t1.header.frame_id = 'world'
            t1.child_frame_id = 'base_footprint'
            t1.transform.translation.x = self.ugv_pose.position.x
            t1.transform.translation.y = self.ugv_pose.position.y
            t1.transform.translation.z = self.ugv_pose.position.z
            t1.transform.rotation = self.ugv_pose.orientation
            self.br.sendTransform(t1)

        if self.uav_pose is not None:
            t2 = TransformStamped()
            t2.header.stamp = now
            t2.header.frame_id = 'world'
            t2.child_frame_id = 'uav_base_link'
            t2.transform.translation.x = self.uav_pose.position.x
            t2.transform.translation.y = self.uav_pose.position.y
            t2.transform.translation.z = self.uav_pose.position.z
            t2.transform.rotation = self.uav_pose.orientation
            self.br.sendTransform(t2)

def main():
    rclpy.init()
    node = DynamicTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
