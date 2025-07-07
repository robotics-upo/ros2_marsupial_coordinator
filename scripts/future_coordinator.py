#!/usr/bin/env python3
import math
import rclpy, os
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import yaml
import time

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.declare_parameter('yaml_filename', 'trajectory.yaml')
        self.yaml_filename = self.get_parameter('yaml_filename').get_parameter_value().string_value

        self.declare_parameter('N', 0)
        self.N_lookahead = self.get_parameter('N').get_parameter_value().integer_value

        self.get_logger().info(f"Coordinador por distancia de ventana {self.N_lookahead} iniciado")

        # Inicialización de parámetros
        self.v_nominal = 0.5
        self.max_speed = 2.0
        self.t_min = 1.0

        # Variables de estado
        self.waypoints_uav = self.load_waypoints_UAV()
        self.waypoints_ugv = self.load_waypoints_UGV()
        self.waypoints_length = self.load_waypoints_length()

        self.current_uav_wp = None
        self.current_ugv_wp = None
        self.current_uav_pose = None
        self.current_ugv_pose = None

        self.uav_reached = False
        self.ugv_reached = False
        self.finished_trajectory_logged = False

        self.initial_wp_uav = 0
        self.initial_wp_ugv = 0

        self.v_uav_xy = 0.0
        self.v_uav_z = 0.0
        self.v_ugv = 0.0

        self.coordinate = True

        # Subscripciones
        self.create_subscription(Pose, 'sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        self.create_subscription(Pose, 'rs_robot/ugv_gt_pose', self.ugv_pose_callback, 10)
        self.create_subscription(Int32, '/uav/waypoint_index', self.uav_wp_callback, 10)
        self.create_subscription(Int32, '/ugv/waypoint_index', self.ugv_wp_callback, 10)

        # Publicadores
        self.uav_speed_pub = self.create_publisher(Float32, "/uav/reference_speed_xy", 10)
        self.uav_speed_pub_z = self.create_publisher(Float32, "/uav/reference_speed_z", 10)
        self.ugv_speed_pub = self.create_publisher(Float32, "/ugv/reference_speed", 10)

        # Timer
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1, self.debug_callback)

    # -------------------- CALLBACKS --------------------
    def uav_pose_callback(self, msg):
        self.current_uav_pose = msg.position

    def ugv_pose_callback(self, msg):
        self.current_ugv_pose = msg.position

    def uav_wp_callback(self, msg):
        self.current_uav_wp = msg.data

    def ugv_wp_callback(self, msg):
        self.current_ugv_wp = msg.data

    # -------------------- CARGA DE YAML --------------------
    def load_yaml_data(self):
        package_share_directory = get_package_share_directory('ros2_marsupial_coordinator')
        yaml_file_path = os.path.join(package_share_directory, 'config', self.yaml_filename)
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"Archivo YAML no encontrado: {yaml_file_path}")
            return
        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        return yaml_data

    def load_waypoints_length(self):
        yaml_data = self.load_yaml_data()
        tether_data = yaml_data['tether']
        waypoint_list = []
        poses_keys = sorted([k for k in tether_data.keys() if k.startswith("length")], key=lambda k: int(k[6:]))
        for key in poses_keys:
            l = tether_data[key]['length']
            waypoint_list.append(l)
        return waypoint_list

    def load_waypoints_UGV(self):
        yaml_data = self.load_yaml_data()
        ugv_data = yaml_data['marsupial_ugv']
        waypoint_list = []
        poses_keys = sorted([k for k in ugv_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
        for key in poses_keys:
            pos = ugv_data[key]['pose']['position']
            waypoint_list.append((pos['x'], pos['y']))
        return waypoint_list

    def load_waypoints_UAV(self):
        yaml_data = self.load_yaml_data()
        uav_data = yaml_data['marsupial_uav']
        waypoint_list = []
        poses_keys = sorted([k for k in uav_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
        for key in poses_keys:
            pos = uav_data[key]['pose']['position']
            waypoint_list.append((pos['x'], pos['y'], pos['z']))
        return waypoint_list

    # -------------------- COMPUTO DE DISTANCIAS --------------------
    def compute_lookahead_distance(self, waypoints, initial_wp):
        total_distance_xy = 0.0
        total_distance_z = 0.0

        wp = waypoints[initial_wp]

        #if(self.N_lookahead > 0): wp = waypoints[initial_wp]
        #elif(self.N_lookahead == 0): wp = waypoints[self.final_wp]

        # Distancia entre la posición actual hasta el waypoint inicial
        if len(wp) == 3:
            dx = wp[0] - self.current_uav_pose.x
            dy = wp[1] - self.current_uav_pose.y
            dz = wp[2] - self.current_uav_pose.z
            total_distance_xy += math.sqrt(dx**2 + dy**2)
            total_distance_z += abs(dz)
        elif len(wp) == 2:
            dx = wp[0] - self.current_ugv_pose.x
            dy = wp[1] - self.current_ugv_pose.y
            total_distance_xy += math.sqrt(dx**2 + dy**2)

        # Ahora sumamos las distancias entre los siguientes waypoints
        #if self.N_lookahead > 0:
        for i in range(initial_wp, min(self.final_wp, len(waypoints)-1)):
            wp_current = waypoints[i]
            wp_next = waypoints[i+1]

                # Si es UAV (3D)
            if len(wp_current) == 3:
                dx = wp_next[0] - wp_current[0]
                dy = wp_next[1] - wp_current[1]
                dz = wp_next[2] - wp_current[2]
                total_distance_xy += math.sqrt(dx**2 + dy**2)
                total_distance_z += abs(dz)

                # Si es UGV (2D)
            elif len(wp_current) == 2:
                dx = wp_next[0] - wp_current[0]
                dy = wp_next[1] - wp_current[1]
                total_distance_xy += math.sqrt(dx**2 + dy**2)

        return total_distance_xy, total_distance_z

    # -------------------- COMPROBACIONES DE LLEGADAS --------------------
    def check_ugv_reached(self):
        if self.initial_wp_ugv is None:
            return False
        if self.final_wp >= len(self.waypoints_ugv):
            return True

        wp_ugv_la = self.waypoints_ugv[self.final_wp]
        dist_ugv = math.hypot(wp_ugv_la[0] - self.current_ugv_pose.x, wp_ugv_la[1] - self.current_ugv_pose.y)

        if dist_ugv < 0.5:
            self.ugv_reached = True

        return dist_ugv < 0.5

    def check_uav_reached(self):
        if self.initial_wp_uav is None:
            return False
        if self.final_wp >= len(self.waypoints_uav):
            return True

        wp_uav_la = self.waypoints_uav[self.final_wp]
        dist_xy = math.hypot(wp_uav_la[0] - self.current_uav_pose.x, wp_uav_la[1] - self.current_uav_pose.y)
        dist_z = abs(wp_uav_la[2] - self.current_uav_pose.z)

        if dist_xy < 0.5 and dist_z < 0.5:
            self.uav_reached = True

        return dist_xy < 0.5 and dist_z < 0.5
    
    def debug_callback(self):
        if(self.current_uav_wp is not None and self.current_ugv_wp):
            self.get_logger().info("------ ESTADO ACTUAL ------")
            self.get_logger().info(f"WP actual de UAV: {self.current_uav_wp}")
            self.get_logger().info(f"WP actual de UGV: {self.current_ugv_wp}")
            self.get_logger().info(f"ERROR: {abs(self.current_uav_wp-self.current_ugv_wp)}")
            self.get_logger().info("-----------------------------")

    # -------------------- CALCULO DE VELOCIDADES --------------------
    def vel_sync(self):

        # Cada vehículo tiene su waypoint inicial, pero el final es común

        if self.current_uav_wp >= self.current_ugv_wp:
            
            if(self.N_lookahead == 0): self.final_wp = self.current_uav_wp
            if(self.N_lookahead > 0): self.final_wp = self.current_uav_wp + self.N_lookahead

        elif(self.current_ugv_wp > self.current_uav_wp):

            if(self.N_lookahead == 0): self.final_wp = self.current_ugv_wp
            if(self.N_lookahead > 0): self.final_wp = self.current_ugv_wp + self.N_lookahead

        if self.final_wp >= len(self.waypoints_ugv):
            self.get_logger().info("Misión completada. Todos los waypoints han sido procesados.")
            rclpy.shutdown()
            return

        self.initial_wp_uav = self.current_uav_wp
        self.initial_wp_ugv = self.current_ugv_wp

        # Calcular distancias de lookahead
        D_ugv, _ = self.compute_lookahead_distance(self.waypoints_ugv, self.initial_wp_ugv)
        D_uav_xy, D_uav_z = self.compute_lookahead_distance(self.waypoints_uav, self.initial_wp_uav)

        max_distance = max(D_ugv, D_uav_xy)
        t_objetivo = max(max_distance / self.v_nominal, self.t_min)

        # Calculamos velocidades
        self.v_ugv = min(D_ugv / t_objetivo, self.max_speed)
        self.v_uav_xy = min(D_uav_xy / t_objetivo, self.max_speed)
        self.v_uav_z = min(D_uav_z / t_objetivo, self.max_speed)

        self.uav_speed_pub.publish(Float32(data=self.v_uav_xy))
        self.uav_speed_pub_z.publish(Float32(data=self.v_uav_z))
        self.ugv_speed_pub.publish(Float32(data=self.v_ugv))

        self.get_logger().info("------ SINCRONIZACIÓN ------")
        self.get_logger().info(f"WP de SINCRONIZACIÓN: {self.final_wp}")
        self.get_logger().info(f"WPs iniciales al calcular -> UAV: {self.initial_wp_uav} - UGV: {self.initial_wp_ugv}")
        self.get_logger().info(f"WPs currents -> UAV: {self.current_uav_wp} - UGV: {self.current_ugv_wp}")
        self.get_logger().info(f"Distancias -> UGV: {D_ugv:.2f} m, UAV XY: {D_uav_xy:.2f} m, UAV Z: {D_uav_z:.2f} m")
        self.get_logger().info(f"T_estimado: {t_objetivo:.2f} s")
        self.get_logger().info(f"Velocidades -> UGV: {self.v_ugv:.2f}, UAV XY: {self.v_uav_xy:.2f}, UAV Z: {self.v_uav_z:.2f}")
        self.get_logger().info("-----------------------------")

        self.coordinate = False 

    # -------------------- BUCLE DE CONTROL PRINCIPAL -------- 
    def control_loop(self):
        if None in [self.current_uav_pose, self.current_ugv_pose, self.current_uav_wp, self.current_ugv_wp]:
            self.get_logger().info("Esperando datos iniciales...")
            return

        if self.coordinate:
            self.get_logger().warn("Calculando sincronización...")
            self.vel_sync()

        else:

            self.check_uav_reached()
            self.check_ugv_reached()

            # UAV control
            if self.uav_reached:
                    self.get_logger().info("UAV ha llegado al WP de sincronización -> NUEVA sincronización")
                    self.coordinate = True
                    self.uav_reached = False
            else:
                self.uav_speed_pub.publish(Float32(data=self.v_uav_xy))
                self.uav_speed_pub_z.publish(Float32(data=self.v_uav_z))

            # UGV control
            if self.ugv_reached:
                    self.get_logger().info("UGV ha llegado al WP de sincronización -> NUEVA sincronización")
                    self.coordinate = True
                    self.ugv_reached = False
            else:
                self.ugv_speed_pub.publish(Float32(data=self.v_ugv))
      

def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
