#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Point
import sys

class WaypointPublisher(Node):
    def __init__(self, yaml_filename='trajectory.yaml'):
        super().__init__('waypoint_publisher')
        self.yaml_filename = yaml_filename
        self.publisher_ = self.create_publisher(MarkerArray, 'waypoints', 100)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.load_waypoints()

    def load_waypoints(self):
 
        package_share_directory = get_package_share_directory('ros2_marsupial_coordinator')
        yaml_file_path = os.path.join(package_share_directory, 'config', self.yaml_filename)

        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"Archivo YAML no encontrado: {yaml_file_path}")
            return

        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        # === UGV ===
        ugv_data = yaml_data['marsupial_ugv']
        ugv_keys = sorted([k for k in ugv_data if k.startswith('poses') and k[5:].isdigit()],
                        key=lambda x: int(x[5:]))
        self.waypoints_ugv = [(ugv_data[k]['pose']['position']['x'],
                            ugv_data[k]['pose']['position']['y']) for k in ugv_keys]

        # === UAV ===
        uav_data = yaml_data['marsupial_uav']
        uav_keys = sorted([k for k in uav_data if k.startswith('poses') and k[5:].isdigit()],
                        key=lambda x: int(x[5:]))
        self.waypoints_uav = [(uav_data[k]['pose']['position']['x'],
                            uav_data[k]['pose']['position']['y'],
                            uav_data[k]['pose']['position']['z']) for k in uav_keys]

        # === Cable Length ===
        tether_data = yaml_data['tether']
        tether_keys = sorted([k for k in tether_data if k.startswith('length') and k[6:].isdigit()],
                            key=lambda x: int(x[6:]))
        self.waypoints_length = [tether_data[k]['length'] for k in tether_keys]

    def timer_callback(self):
        marker_array = MarkerArray()
        marker_id = 0

        for i in range(min(len(self.waypoints_ugv), len(self.waypoints_uav))):
            ugv_x, ugv_y = self.waypoints_ugv[i]
            uav_x, uav_y, uav_z = self.waypoints_uav[i]

            # === UGV Marker (2D) ===
            ugv_marker = Marker()
            ugv_marker.header.frame_id = "world"
            ugv_marker.header.stamp = self.get_clock().now().to_msg()
            ugv_marker.ns = "ugv"
            ugv_marker.id = marker_id
            marker_id += 1
            ugv_marker.type = Marker.SPHERE
            ugv_marker.action = Marker.ADD
            ugv_marker.pose.position.x = ugv_x
            ugv_marker.pose.position.y = ugv_y
            ugv_marker.pose.position.z = 0.0
            ugv_marker.scale.x = 0.1
            ugv_marker.scale.y = 0.1
            ugv_marker.scale.z = 0.1
            ugv_marker.color.r = 1.0
            ugv_marker.color.g = 0.0
            ugv_marker.color.b = 0.0
            ugv_marker.color.a = 1.0
            marker_array.markers.append(ugv_marker)

            # === UAV Marker (3D) ===
            uav_marker = Marker()
            uav_marker.header.frame_id = "world"
            uav_marker.header.stamp = self.get_clock().now().to_msg()
            uav_marker.ns = "uav"
            uav_marker.id = marker_id
            marker_id += 1
            uav_marker.type = Marker.SPHERE
            uav_marker.action = Marker.ADD
            uav_marker.pose.position.x = uav_x
            uav_marker.pose.position.y = uav_y
            uav_marker.pose.position.z = uav_z
            uav_marker.scale.x = 0.1
            uav_marker.scale.y = 0.1
            uav_marker.scale.z = 0.1
            uav_marker.color.r = 0.0
            uav_marker.color.g = 0.0
            uav_marker.color.b = 1.0
            uav_marker.color.a = 1.0
            marker_array.markers.append(uav_marker)

            # === Cable Marker ===
            cable_marker = Marker()
            cable_marker.header.frame_id = "world"
            cable_marker.header.stamp = self.get_clock().now().to_msg()
            cable_marker.ns = "cable"
            cable_marker.id = marker_id
            marker_id += 1
            cable_marker.type = Marker.LINE_STRIP
            cable_marker.action = Marker.ADD
            cable_marker.scale.x = 0.01
            cable_marker.color.r = 0.0
            cable_marker.color.g = 1.0
            cable_marker.color.b = 0.0
            cable_marker.color.a = 1.0

            p1 = Point(x=ugv_x, y=ugv_y, z=0.0)
            p2 = Point(x=uav_x, y=uav_y, z=uav_z)
            cable_marker.points.append(p1)
            cable_marker.points.append(p2)
            marker_array.markers.append(cable_marker)

            self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    # Verifica si se pasó un argumento de archivo YAML
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = 'trajectory.yaml'  # Valor por defecto

    waypoint_publisher = WaypointPublisher(yaml_file)
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()