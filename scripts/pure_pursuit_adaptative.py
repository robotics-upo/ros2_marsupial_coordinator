#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import sys
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from pycatenary import cable
from math import sqrt

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

            if i > 0 and self.waypoints[i] == self.waypoints[i - 1]:
                self.current_index = i+1
                return self.waypoints[i]

        self.current_index = len(self.waypoints) - 1
        return self.waypoints[-1]

    def find_lookahead_point_3D(self, current_position):
        for i in range(self.current_index, len(self.waypoints)):
            dx = self.waypoints[i][0] - current_position[0]
            dy = self.waypoints[i][1] - current_position[1]
            dz = self.waypoints[i][2] - current_position[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist >= self.lookahead_distance:
                self.current_index = i
                return self.waypoints[i]

            if i > 0 and self.waypoints[i] == self.waypoints[i - 1]:
                self.current_index = i+1
                return self.waypoints[i]

        self.current_index = len(self.waypoints) - 1
        return self.waypoints[-1]
    
    def has_finished(self):
        return self.current_index >= len(self.waypoints) - 1

    def get_current_index(self):
        return self.current_index 
    
    def get_lookahead_distance(self):
        return self.lookahead_distance
    
class UGV_UAV_PurePursuitNode(Node):
    def __init__(self, yaml_filename='trajectory.yaml'):
        super().__init__('ugv_uav_pure_pursuit_node')

        self.yaml_filename = yaml_filename

        self.current_pose_uav = None
        self.current_pose_ugv = None

        self.ugv_reached = False
        self.uav_reached = False

        self.last_marker_array = None

        # Comportamiento del lookahead adaptativo UAV
        self.lookahead_uav_min = 0.3
        self.lookahead_uav_max = 1.0
        self.lookahead_decay_rate = 0.3   # penalizacion

        # Comportamiento del lookahead adaptativo del UGV
        self.lookahead_ugv_min = 0.3
        self.lookahead_ugv_max = 1.0
        self.lookahead_decay_rate_ugv = 0.3

        # Threshold de wp antes de penalizar
        self.lookahead_threshold = 2

        # Indices de los waypoints 
        self.ugv_wp_index = 0
        self.uav_wp_index = 0
        self.tether_wp_index = 0

        # Guardamos en una lista todas las coordenadas de todos los waypoints, para UGV, UAV y CABLE (tether)
        self.waypoints_ugv = self.load_waypoints_UGV()
        self.waypoints_uav = self.load_waypoints_UAV()
        self.waypoints_tether = self.load_waypoints_length()
        
        # Pasamos esa lista a nuestro pure pursuit para que se inicie correctamente (con una lh distance inicial en 0.5m en este caso)
        self.pure_pursuit_ugv = PurePursuit(self.waypoints_ugv, lookahead_distance=0.5)
        self.pure_pursuit_uav = PurePursuit(self.waypoints_uav, lookahead_distance=0.5)

        # === PUBLICADORES ===

        # Radios visibles de la lookahead distance
        self.pub_radius_ugv = self.create_publisher(Marker, '/ugv/radius', 10)
        self.pub_radius_uav = self.create_publisher(Marker, '/uav/radius', 10)

        # Publicador para ver el lookahead point UGV, UAV y CABLE que se está siguiendo
        self.lookahead_marker_pub_ugv = self.create_publisher(Marker, '/ugv/lookahead_marker', 10)
        self.lookahead_marker_pub_uav = self.create_publisher(Marker, '/uav/lookahead_marker', 10)
        self.publisher_cable_pub = self.create_publisher(MarkerArray, '/tether/lookahead_cable', 10)
        self.lookahead_marker_pub_ugv_winch = self.create_publisher(Marker, '/winch/lookahead_marker', 10)

        # Publicadores de los índices (j, k, m) actuales de waypoint tanto del tether, como UGV, como UAV
        self.wp_uav_index_publisher = self.create_publisher(Int32, '/uav/waypoint_index', 10)
        self.wp_ugv_index_publisher = self.create_publisher(Int32, '/ugv/waypoint_index', 10)
        self.wp_tether_index_publisher = self.create_publisher(Int32, '/tether/waypoint_index', 10)
        
        # === SUSCRIPCIONES ===
        
        # Suscripción al topic que informa de la posición gt del UGV y UAV
        self.create_subscription(Pose, 'rs_robot/ugv_gt_pose', self.pose_callback_ugv, 10)
        self.create_subscription(Pose, 'sjtu_drone/gt_pose', self.pose_callback_uav, 10)

        # Publicador del punto de referencia para los miembros del sistema marsupial
        self.reference_pub_ugv = self.create_publisher(Pose, '/ugv/reference_pose', 10)
        self.reference_pub_uav = self.create_publisher(Pose, '/uav/reference_pose', 10)
        self.pub_reference_length = self.create_publisher(Float64, '/tether/reference_length', 10)

        # Información de los marcadores de waypoints ya publicados (obtener datos de las catenarias)
        self.subscriber_ = self.create_subscription(MarkerArray, 'waypoints', self.marker_callback, 10)

    # === CALLBACK DE WAYPOINTS PARA ACCESO A CATENARIAS ===
    def marker_callback(self, msg):
        self.last_marker_array = msg

    # === CÁLCULO DE LOOKAHEAD ADAPTATIVO ===
    def compute_adaptive_lookahead_uav(self):
        delta_wp = self.uav_wp_index - self.ugv_wp_index
        delta_wp = max(0, delta_wp)
        decay = math.exp(-self.lookahead_decay_rate * delta_wp)
        lookahead = self.lookahead_uav_min + (self.lookahead_uav_max - self.lookahead_uav_min) * decay

        return lookahead
    
    def compute_adaptive_lookahead_ugv(self):
        delta_wp = self.ugv_wp_index - self.uav_wp_index
        delta_wp = max(0, delta_wp)
        decay = math.exp(-self.lookahead_decay_rate_ugv * delta_wp)
        lookahead = self.lookahead_ugv_min + (self.lookahead_ugv_max - self.lookahead_ugv_min) * decay

        return lookahead

    # === CARGA DE WAYPOINTS DE LA TRAYECTORIA ===
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

        poses_keys = [k for k in tether_data.keys() if k.startswith("length") and k[6:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[6:]))

        for key in poses_keys_sorted:
            l = tether_data[key]['length']
            waypoint_list.append(float(l))

        return waypoint_list

    def load_waypoints_UGV(self):
        yaml_data = self.load_yaml_data(self.yaml_filename)

        ugv_data = yaml_data['marsupial_ugv']
        waypoint_list = []

        poses_keys = [k for k in ugv_data.keys() if k.startswith("poses") and k[5:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[5:]))

        for key in poses_keys_sorted:
            pos = ugv_data[key]['pose']['position']
            waypoint_list.append((float(pos['x']), float(pos['y'])))

        return waypoint_list
    
    def load_waypoints_UAV(self):
        yaml_data = self.load_yaml_data(self.yaml_filename)

        uav_data = yaml_data['marsupial_uav']
        waypoint_list = []

        poses_keys = [k for k in uav_data.keys() if k.startswith("poses") and k[5:].isdigit()]
        poses_keys_sorted = sorted(poses_keys, key=lambda k: int(k[5:]))

        for key in poses_keys_sorted:
            pos = uav_data[key]['pose']['position']
            waypoint_list.append((float(pos['x']), float(pos['y']), float(pos['z'])))

        return waypoint_list
    
    # === PUBLICADOR DE LOOKAHEAD POINT ===
    def publish_lookahead_marker(self, lookahead, is_3d, namespace, marker_id, publisher):

        # Offset en los ejes x y z del UGV para hallar el centro del cabrestante
        offset_ugv_z = 0.9
        offset_ugv_x = -0.5

        # Marcador
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posición 
        marker.pose.position.x = lookahead[0]
        marker.pose.position.y = lookahead[1]
        marker.pose.position.z = lookahead[2] if is_3d else 0.0

        # En caso de ser el marcador de cabrestante se computa el offset
        if(marker_id==3): 
            marker.pose.position.z = offset_ugv_z
            marker.pose.position.x = lookahead[0] + offset_ugv_x

        # Tamaño
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # En caso de ser el marcador de cabrestante se computa el offset
        if(marker_id==3):
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

        # Color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        publisher.publish(marker)

    # === PUBLICADOR DE LOOKAHEAD DISTANCE ===
    def publish_radius_marker(self, radius, frame_id, publisher, namespace):

        # Marcador
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posición centrada en el origen del frame local
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Diámetro del marcador en cada eje
        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 2 * radius

        # Color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        marker.lifetime.sec = 0
        marker.frame_locked = False

        publisher.publish(marker)

    # === CÁLCULO DE REFERENCIA DEL CABLE ===
    def update_tether_index(self):

        # El controlador del cable recibe una referencia en longitud, y concretamente se le pasará como longitud 
        # objetivo aquella almacenada en el índice (self.tether_wp_index) de la lista de waypoints tether cargado

        # === VERSION 1 ===
        # self.tether_wp_index = math.ceil((self.uav_wp_index + self.ugv_wp_index)/2)

        # === VERSION 2 ===
        # if self.current_pose_uav is None or self.current_pose_ugv is None:
        #     return  

        # dx = self.current_pose_uav[0] - self.current_pose_ugv[0]
        # dy = self.current_pose_uav[1] - self.current_pose_ugv[1]
        # dz = self.current_pose_uav[2] - self.current_pose_ugv[2]
        
        # d_total = math.sqrt(dx**2 + dy**2 + dz**2)

        # closest_index = min(range(len(self.waypoints_tether)), key=lambda i: abs(self.waypoints_tether[i] - d_total))
        # self.tether_wp_index = closest_index

        # === VERSION FINAL ===
        self.tether_wp_index = self.uav_wp_index

        if self.last_marker_array is None:
            return

        # === LOOKAHEAD CABLE ===
        modified_marker_array = MarkerArray()
        modified_id = -1

        for marker in self.last_marker_array.markers:
            if marker.ns == "cable" and marker.id == self.tether_wp_index:

                modified_marker = Marker()
                modified_marker.header = marker.header
                modified_marker.ns = "lh_cable"
                modified_marker.id = modified_id
                modified_marker.type = marker.type
                modified_marker.action = Marker.ADD
                modified_marker.pose = marker.pose
                modified_marker.points = marker.points
                
                modified_marker.scale.x = 0.025  # Grosor del cable
                modified_marker.color.r = 0.0
                modified_marker.color.g = 1.0
                modified_marker.color.b = 0.0
                modified_marker.color.a = 1.0

                modified_marker_array.markers.append(modified_marker)

        if modified_marker_array.markers:
            self.publisher_cable_pub.publish(modified_marker_array)

    # === CALLBACK UGV ===
    def pose_callback_ugv(self, msg):

        # Comprobamos si hemos terminado
        if self.pure_pursuit_ugv.has_finished():
            self.get_logger().info('El UGV ha finalizado su trayectoria.')
            self.ugv_reached = True

        # Actualizo posición actual
        current_position = (msg.position.x, msg.position.y)
        self.current_pose_ugv = (msg.position.x, msg.position.y, 0.0)
        
        # Calculo índice objetivo del cable
        self.update_tether_index()
        target_length = Float64()
        target_length.data = self.waypoints_tether[self.tether_wp_index]  
        self.pub_reference_length.publish(target_length)

        # Crea y publica mensaje
        msg = Int32()
        msg.data = self.tether_wp_index
        self.wp_tether_index_publisher.publish(msg)

        # Ajusta dinámicamente el lookahead según la distancia entre UGV y UAV
        if not self.ugv_reached:
            adaptive_lookahead_ugv = self.compute_adaptive_lookahead_ugv()
            self.pure_pursuit_ugv.lookahead_distance = adaptive_lookahead_ugv

        lookahead = self.pure_pursuit_ugv.find_lookahead_point_2D(current_position)

        # Crea el mensaje de referencia
        ref_pose = Pose()
        ref_pose.position.x = lookahead[0]
        ref_pose.position.y = lookahead[1]
        ref_pose.position.z = 0.0
        ref_pose.orientation.w = 1.0 # Orientación como identidad

        # Publicación de referencia para el controlador del UGV
        self.reference_pub_ugv.publish(ref_pose)

        # Publicación de marcadores visuales
        self.publish_lookahead_marker(lookahead, is_3d=False, namespace="ugv_lookahead", marker_id=1, publisher=self.lookahead_marker_pub_ugv)
        self.publish_lookahead_marker(lookahead, is_3d=False, namespace="ugv_winch_lookahead", marker_id=3, publisher=self.lookahead_marker_pub_ugv_winch)

        # Actualización del índice de waypoint UGV
        self.ugv_wp_index = self.pure_pursuit_ugv.get_current_index()
        msg = Int32()
        msg.data = self.ugv_wp_index
        self.wp_ugv_index_publisher.publish(msg)

        # Publicación de radios como lookahead distance del UGV
        radius = self.pure_pursuit_ugv.get_lookahead_distance()
        self.publish_radius_marker(radius, "base_link", self.pub_radius_ugv, "ugv_sphere")

    # === CALLBACK UAV ===
    def pose_callback_uav(self, msg):

        # Comprobamos si hemos terminado
        if self.pure_pursuit_uav.has_finished():
            self.get_logger().info('El UAV ha finalizado su trayectoria.')

        # Actualizo posición actual
        current_position = (msg.position.x, msg.position.y, msg.position.z)
        self.current_pose_uav = (msg.position.x, msg.position.y, msg.position.z)

        # Ajusta dinámicamente el lookahead según la distancia entre UAV y UGV
        if not self.uav_reached:
            adaptive_lookahead = self.compute_adaptive_lookahead_uav()
            self.pure_pursuit_uav.lookahead_distance = adaptive_lookahead
            lookahead = self.pure_pursuit_uav.find_lookahead_point_3D(current_position)

        # Crea el mensaje de referencia
        ref_pose = Pose()
        ref_pose.position.x = lookahead[0]
        ref_pose.position.y = lookahead[1]
        ref_pose.position.z = lookahead[2]
        ref_pose.orientation.w = 1.0 # Orientación como identidad

        # Publicación de referencia para el controlador UAV
        self.reference_pub_uav.publish(ref_pose)
        self.publish_lookahead_marker(lookahead, is_3d=True, namespace="uav_lookahead", marker_id=2, publisher=self.lookahead_marker_pub_uav)

        # Actualizo el índice de waypoint UAV
        self.uav_wp_index = self.pure_pursuit_uav.get_current_index()
        msg = Int32()
        msg.data = self.uav_wp_index
        self.wp_uav_index_publisher.publish(msg)

        # Publicación de radios como lookahead distance del UAV
        radius = self.pure_pursuit_uav.get_lookahead_distance()
        self.publish_radius_marker(radius, "uav_base_link", self.pub_radius_uav, "uav_sphere")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        yaml_file = 'trajectory.yaml' 

    node = UGV_UAV_PurePursuitNode(yaml_file)

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()  
        rclpy.shutdown()

if __name__ == '__main__':
    main()