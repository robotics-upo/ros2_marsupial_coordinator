#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Point
import sys
from math import sqrt
from pycatenary import cable
import numpy as np
from std_msgs.msg import Float32MultiArray

class WaypointPublisher(Node):
    def __init__(self, yaml_filename='trajectory.yaml'):
        super().__init__('waypoint_publisher')

        # Publicador de las longitudes objetivo modificadas para garantizarse el calculo de catenarias
        self.waypoints_length_pub = self.create_publisher(Float32MultiArray, 'modified_tether_length', 10)
 
        self.ugv_initial_height = 0.275  # Altura del UGV
        self.precomputed_markers = None
        self.yaml_filename = yaml_filename
        self.publisher_ = self.create_publisher(MarkerArray, 'waypoints', 100)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.load_waypoints()
        self.precomputed_markers = self.compute_all_markers()
        self.publish_waypoints_length()

    def publish_waypoints_length(self):
        msg = Float32MultiArray()
        msg.data = self.waypoints_length  
        self.waypoints_length_pub.publish(msg)

    # === CÓMPUTO DE TODOS LOS WAYPOINTS ===
    def compute_all_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        marker_cable_id = 0

        for i in range(min(len(self.waypoints_ugv), len(self.waypoints_uav))):
            ugv_x, ugv_y, ugv_z = self.waypoints_ugv[i]
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
            ugv_marker.pose.position.z = ugv_z
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

            # === Offset UGV-WINCH Marker ===
            offset_ugv_z = 0.35
            offset_ugv_x = -0.25

            offset_marker = Marker()
            offset_marker.header.frame_id = "world"
            offset_marker.header.stamp = self.get_clock().now().to_msg()
            offset_marker.ns = "ugv_offset"
            offset_marker.id = marker_id
            marker_id += 1
            offset_marker.type = Marker.SPHERE
            offset_marker.action = Marker.ADD
            offset_marker.pose.position.x = float(ugv_x + offset_ugv_x)
            offset_marker.pose.position.y = float(ugv_y)
            offset_marker.pose.position.z = float(offset_ugv_z + self.ugv_initial_height)
            offset_marker.scale.x = 0.1
            offset_marker.scale.y = 0.1
            offset_marker.scale.z = 0.1
            offset_marker.color.r = 1.0
            offset_marker.color.g = 0.5
            offset_marker.color.b = 0.0
            offset_marker.color.a = 1.0
            marker_array.markers.append(offset_marker)

            # === Catenaria ===
            anchor_point = (ugv_x + offset_ugv_x, ugv_y, offset_ugv_z + self.ugv_initial_height)
            fairlead_point = (uav_x, uav_y, uav_z)
            cable_length = self.waypoints_length[i]

            dx = fairlead_point[0] - anchor_point[0]
            dy = fairlead_point[1] - anchor_point[1]
            dz = fairlead_point[2] - anchor_point[2]
            distance = sqrt(dx**2 + dy**2 + dz**2)

            if distance >= cable_length:
                #self.get_logger().warn(f"[Waypoint {i}] Distancia ({distance:.2f} m) >= Longitud cuerda ({cable_length:.2f} m): NO se puede formar una catenaria")
                cable_length = distance + 0.3
                self.waypoints_length[i] = cable_length

            catenary_points = self.generate_catenary_points(anchor_point, fairlead_point, cable_length, i, distance)

            if not catenary_points:
                #self.get_logger().warn(f"[Waypoint {i}] Catenaria vacía: se omite la visualización de este cable.")
                continue

            cable_marker = Marker()
            cable_marker.header.frame_id = "world"
            cable_marker.header.stamp = self.get_clock().now().to_msg()
            cable_marker.ns = "cable"
            cable_marker.id = marker_cable_id
            marker_cable_id += 1
            cable_marker.type = Marker.LINE_STRIP
            cable_marker.action = Marker.ADD
            cable_marker.scale.x = 0.016
            cable_marker.color.r = 1.0
            cable_marker.color.g = 0.0
            cable_marker.color.b = 0.0
            cable_marker.color.a = 1.0
            cable_marker.points = catenary_points
            marker_array.markers.append(cable_marker)

            self.publisher_.publish(marker_array)

        return marker_array

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
        self.waypoints_ugv = [(float(ugv_data[k]['pose']['position']['x']), float(ugv_data[k]['pose']['position']['y']), self.ugv_initial_height) for k in ugv_keys]
        
        self.get_logger().info('UGV waypoints cargados:')
        self.get_logger().info(f"{self.waypoints_ugv}")

        # === UAV ===
        uav_data = yaml_data['marsupial_uav']
        uav_keys = sorted([k for k in uav_data if k.startswith('poses') and k[5:].isdigit()],
                        key=lambda x: int(x[5:]))
        self.waypoints_uav = [(float(uav_data[k]['pose']['position']['x']), float(uav_data[k]['pose']['position']['y']), float(uav_data[k]['pose']['position']['z'])) for k in uav_keys]
        
        self.get_logger().info('UAV waypoints cargados:')
        self.get_logger().info(f"{self.waypoints_uav}")

        # === CABLE ===
        tether_data = yaml_data['tether']
        tether_keys = sorted([k for k in tether_data if k.startswith('length') and k[6:].isdigit()],
                            key=lambda x: int(x[6:]))
        self.waypoints_length = [float(tether_data[k]['length']) for k in tether_keys]

        self.get_logger().info('Tether waypoints cargados:')
        self.get_logger().info(f"{self.waypoints_length}")

        self.get_logger().info(f'WAYPONTS TOTALES: {len(self.waypoints_length)}, {len(self.waypoints_ugv)}, {len(self.waypoints_uav)}')

    # === CÁLCULO DE LA CATENARIA ===
    def generate_catenary_points(self, anchor, fairlead, length, indice_wp, dist, EA=None, floor=False, resolution=103):
        l_cable = 0.1  # m
        m = 0.001
        w = l_cable / m

        try:
            l = cable.MooringLine(L=length, w=w, EA=EA, anchor=anchor, fairlead=fairlead, floor=floor)
            l.computeSolution()

            s_values = [i * length / (resolution - 1) for i in range(resolution)]
            xyz_points = [l.s2xyz(min(s, l.L)) for s in s_values]

            points = []

            for x, y, z in xyz_points:
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                points.append(p)

            #self.get_logger().info(f"[Waypoint {indice_wp}] Catenaria calculada con éxito")

            return points

        except Exception as e:
            self.get_logger().error((f"[ERROR] [Waypoint: {indice_wp}] No se pudo calcular la catenaria: {e}"))
            return []

    # === PUBLICAR EN RVIZ DE FORMA PERIÓDICA ===
    def timer_callback(self):
        if self.precomputed_markers:
            self.publisher_.publish(self.precomputed_markers)

        #self.publish_waypoints_length()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = 'trajectory.yaml'

    waypoint_publisher = WaypointPublisher(yaml_file)
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()