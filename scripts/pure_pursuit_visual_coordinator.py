#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
import sys
from std_msgs.msg import Empty
from std_msgs.msg import Int32

class PurePursuit:
    def __init__(self, waypoints, lookahead_distance=1.5):
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.current_index = 0

    def find_lookahead_point_2D(self, current_position):
        for i in range(self.current_index, len(self.waypoints)):
            dx = self.waypoints[i][0] - current_position[0]
            dy = self.waypoints[i][1] - current_position[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist >= self.lookahead_distance:
                self.current_index = i
                return self.waypoints[i]
        return self.waypoints[-1]  # Último punto si ya es el final
    
    def find_lookahead_point_3D(self, current_position):
        for i in range(self.current_index, len(self.waypoints)):
            dx = self.waypoints[i][0] - current_position[0]
            dy = self.waypoints[i][1] - current_position[1]
            dz = self.waypoints[i][2] - current_position[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist >= self.lookahead_distance:
                self.current_index = i
                return self.waypoints[i]
        return self.waypoints[-1]  # Último punto si ya es el final
    
    def get_current_index(self):
        return self.current_index # INDICE ACTUAL DE WAYPOINT
    
    def get_lookahead_distance(self):
        return self.lookahead_distance
    
class UGV_UAV_PurePursuitNode(Node):
    def __init__(self, yaml_filename='trajectory.yaml'):
        super().__init__('ugv_uav_pure_pursuit_node')

        self.yaml_filename = yaml_filename

        self.timer_debug = self.create_timer(1.0, self.debug_callback)

        # Radios visibles de la lookahead distance
        self.pub_radius_ugv = self.create_publisher(Marker, '/ugv/radius', 10)
        self.pub_radius_uav = self.create_publisher(Marker, '/uav/radius', 10)

        # Indices de los waypoints a 0
        self.ugv_wp_index = 0
        self.uav_wp_index = 0
        self.tether_wp_index = 0
        
        # Publicador para ver el lookahead point UGV y UAV que se está siguiendo
        self.lookahead_marker_pub_ugv = self.create_publisher(Marker, '/ugv/lookahead_marker', 10)
        self.lookahead_marker_pub_uav = self.create_publisher(Marker, '/uav/lookahead_marker', 10)

        # Suscripción al topic que informa de la posición del UGV y UAV
        self.create_subscription(Pose, 'rs_robot/ugv_gt_pose', self.pose_callback_ugv, 10)
        self.create_subscription(Pose, 'sjtu_drone/gt_pose', self.pose_callback_uav, 10)

        # Publicador del punto de referencia
        self.reference_pub_ugv = self.create_publisher(Pose, '/ugv/reference_pose', 10)
        self.reference_pub_uav = self.create_publisher(Pose, '/uav/reference_pose', 10)
        self.pub_cable_length = self.create_publisher(Float64, '/cable_length', 10)

        # Guardamos en una lista todas las coordenadas de todos los waypoints, para UGV y UAV
        self.waypoints_ugv = self.load_waypoints_UGV()
        self.waypoints_uav = self.load_waypoints_UAV()
        self.waypoints_tether = self.load_waypoints_length()
        
        # Pasamos esa lista a nuestro pure pursuit para que se inicie correctamente
        self.pure_pursuit_ugv = PurePursuit(self.waypoints_ugv, lookahead_distance=0.5)
        self.pure_pursuit_uav = PurePursuit(self.waypoints_uav, lookahead_distance=0.5)

        # Publicadores de los índices actuales de waypoint tanto del tether, como UGV, como UAV
        self.wp_uav_index_publisher = self.create_publisher(Int32, '/uav/waypoint_index', 10)
        self.wp_ugv_index_publisher = self.create_publisher(Int32, '/ugv/waypoint_index', 10)
        self.wp_tether_index_publisher = self.create_publisher(Int32, '/tether/waypoint_index', 10)

    def load_yaml_data(self, filename='trajectory.yaml'):

        package_share_directory = get_package_share_directory('ros2_marsupial_coordinator')
        yaml_file_path = os.path.join(package_share_directory, 'config', filename)

        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"Archivo YAML no encontrado: {yaml_file_path}")
            return

        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)


        return yaml_data

    def load_waypoints_length(self):
        yaml_data = self.load_yaml_data(self.yaml_filename)
        tether_data = yaml_data['tether']

        waypoint_list = []

        # Filtra solo claves que empiecen por "poses" y tengan un número al final
        poses_keys = [k for k in tether_data.keys() if k.startswith("length") and k[6:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[6:]))

        for key in poses_keys_sorted:
            l = tether_data[key]['length']
            waypoint_list.append(l)

        return waypoint_list

    def load_waypoints_UGV(self):
        yaml_data = self.load_yaml_data(self.yaml_filename)

        # Convertimos cada punto a una tupla (x, y)
        ugv_data = yaml_data['marsupial_ugv']
        waypoint_list = []

        poses_keys = [k for k in ugv_data.keys() if k.startswith("poses") and k[5:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[5:]))

        for key in poses_keys_sorted:
            pos = ugv_data[key]['pose']['position']
            waypoint_list.append((pos['x'], pos['y']))

        return waypoint_list
    
    def load_waypoints_UAV(self):
        yaml_data = self.load_yaml_data(self.yaml_filename)

        uav_data = yaml_data['marsupial_uav']
        waypoint_list = []

        poses_keys = [k for k in uav_data.keys() if k.startswith("poses") and k[5:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[5:]))

        for key in poses_keys_sorted:
            pos = uav_data[key]['pose']['position']
            waypoint_list.append((pos['x'], pos['y'], pos['z']))

        return waypoint_list
    
    def publish_lookahead_marker_ugv(self, lookahead):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ugv_lookahead"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lookahead[0]
        marker.pose.position.y = lookahead[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.lookahead_marker_pub_ugv.publish(marker)

    def publish_lookahead_marker_uav(self, lookahead):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_lookahead"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lookahead[0]
        marker.pose.position.y = lookahead[1]
        marker.pose.position.z = lookahead[2]
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.lookahead_marker_pub_uav.publish(marker)

    # Cada vez que se actualice el movimiento del UGV se calcula el lookahead point conociendo nuestra posición actual y se publica dicho punto como referencia
    def pose_callback_ugv(self, msg):
        current_position = (msg.position.x, msg.position.y)

        # Calcula el punto de seguimiento
        lookahead = self.pure_pursuit_ugv.find_lookahead_point_2D(current_position)

        # Crea el mensaje de referencia
        ref_pose = Pose()
        ref_pose.position.x = lookahead[0]
        ref_pose.position.y = lookahead[1]
        ref_pose.position.z = 0.0

        # Orientación como identidad
        ref_pose.orientation.w = 1.0 # Orientación nula, (no establecida)

        self.reference_pub_ugv.publish(ref_pose)

        self.publish_lookahead_marker_ugv(lookahead)

        # Actualizo el índice de waypoint por el que voy
        self.ugv_wp_index = self.pure_pursuit_ugv.get_current_index()
        self.tether_wp_index = math.ceil((self.uav_wp_index + self.ugv_wp_index)/2)

        # Protegemos el acceso
        if self.tether_wp_index < len(self.waypoints_tether):
            target_length = Float64()
            target_length.data = self.waypoints_tether[self.tether_wp_index]  
            self.pub_cable_length.publish(target_length)
        else:
            self.get_logger().warn(
                f'Índice de tether fuera de rango: {self.tether_wp_index} >= {len(self.waypoints_tether)}. No se publica longitud.'
            )

        radius = self.pure_pursuit_ugv.get_lookahead_distance()

        marker = Marker()
        marker.header.frame_id = "base_link"  # Marco al que estará fijado
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ugv_sphere"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posición: centrado en el origen del frame
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Escala (diámetro en x, y, z)
        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 2 * radius

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        marker.lifetime.sec = 0
        marker.frame_locked = False

        self.pub_radius_ugv.publish(marker)

    # Cada vez que se actualice el movimiento del UAV se calcula el lookahead point conociendo nuestra posición actual y se publica dicho punto como referencia
    def pose_callback_uav(self, msg):
        current_position = (msg.position.x, msg.position.y, msg.position.z)

        # Calcula el punto de seguimiento
        lookahead = self.pure_pursuit_uav.find_lookahead_point_3D(current_position)

        # Crea el mensaje de referencia
        ref_pose = Pose()
        ref_pose.position.x = lookahead[0]
        ref_pose.position.y = lookahead[1]
        ref_pose.position.z = lookahead[2]

        # Orientación como identidad
        ref_pose.orientation.w = 1.0 # Orientación nula, (no establecida)

        self.reference_pub_uav.publish(ref_pose)

        self.publish_lookahead_marker_uav(lookahead)

        # Actualizo el índice de waypoint por el que voy
        self.uav_wp_index = self.pure_pursuit_uav.get_current_index()
        self.tether_wp_index = math.ceil((self.uav_wp_index + self.ugv_wp_index)/2)

                # Protegemos el acceso
        if self.tether_wp_index < len(self.waypoints_tether):
            target_length = Float64()
            target_length.data = self.waypoints_tether[self.tether_wp_index]  
            self.pub_cable_length.publish(target_length)
        else:
            self.get_logger().warn(
                f'Índice de tether fuera de rango: {self.tether_wp_index} >= {len(self.waypoints_tether)}. No se publica longitud.'
            )

        radius = self.pure_pursuit_uav.get_lookahead_distance()

        marker = Marker()
        marker.header.frame_id = "uav_base_link"  # Marco al que estará fijado
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_sphere"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posición: centrado en el origen del frame
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Escala (diámetro en x, y, z)
        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 2 * radius

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        marker.lifetime.sec = 0
        marker.frame_locked = False

        self.pub_radius_uav.publish(marker)
    
    def debug_callback(self):
        #self.get_logger().info(f'UGV Index: {self.ugv_wp_index}, UAV Index: {self.uav_wp_index}, Tether Index: {self.tether_wp_index}')
        msg = Int32()

        msg.data = self.ugv_wp_index
        self.wp_ugv_index_publisher.publish(msg)

        msg.data = self.tether_wp_index
        self.wp_tether_index_publisher.publish(msg)

        msg.data = self.uav_wp_index
        self.wp_uav_index_publisher.publish(msg)



        ## SALIDAS:               /uav_reference_pose    /ugv_reference_pose   (con el lookahead point)    /cable_length ( ceil((wp_uav+wp_ugv)/2) )

        ## VARIABLES ÚTILES:      self.uav_wp_index      self.ugv_wp_index      self.tether_wp_index

def main(args=None):
    rclpy.init(args=args)

    # Verifica si se pasó un argumento de archivo YAML
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = 'trajectory.yaml'  # Valor por defecto


    node = UGV_UAV_PurePursuitNode(yaml_file)

    try:
        rclpy.spin(node)  # Mantiene el nodo corriendo
    except KeyboardInterrupt:
        pass  # Permite salir con Ctrl+C sin error
    finally:
        node.destroy_node()  # Limpieza del nodo
        rclpy.shutdown()     # Cierra ROS 2

if __name__ == '__main__':
    main()