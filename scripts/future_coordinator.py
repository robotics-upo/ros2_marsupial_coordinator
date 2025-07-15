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

        self.ugv_initial_height = 0.275  # Altura del UGV

        self.declare_parameter('yaml_filename', 'trajectory.yaml')
        self.yaml_filename = self.get_parameter('yaml_filename').get_parameter_value().string_value

        self.declare_parameter('N', 0)
        self.N_lookahead = self.get_parameter('N').get_parameter_value().integer_value

        self.get_logger().info(f"Coordinador por distancia de ventana {self.N_lookahead} iniciado")

        # Inicialización de parámetros
        self.v_min = 0.2         
        self.v_max_uav = 0.5
        self.v_max_ugv = 2.0         

        self.waypoints_uav = self.load_waypoints_UAV()
        self.waypoints_ugv = self.load_waypoints_UGV()

        self.current_uav_wp = None
        self.current_ugv_wp = None
        self.current_uav_pose = None
        self.current_ugv_pose = None

        self.uav_reached = False
        self.ugv_reached = False
        self.finished_trajectory_logged = False

        self.uav_tol = 1.0
        self.ugv_tol = 1.0

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
        self.subscription_ugv = self.create_subscription(Float32, '/ugv/distance', self.ugv_distance_callback, 10)
        self.subscription_uav = self.create_subscription(Float32, '/uav/distance', self.uav_distance_callback, 10)

        # Publicadores
        self.uav_speed_pub = self.create_publisher(Float32, "/uav/reference_speed_xy", 10)
        self.uav_speed_pub_z = self.create_publisher(Float32, "/uav/reference_speed_z", 10)
        self.ugv_speed_pub = self.create_publisher(Float32, "/ugv/reference_speed", 10)

        # Timer
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1, self.debug_callback)

    # -------------------- CALLBACKS --------------------

    def ugv_distance_callback(self, msg):
        self.ugv_tol = msg.data

    def uav_distance_callback(self, msg):
        self.uav_tol = msg.data

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

    def load_waypoints_UGV(self):
        yaml_data = self.load_yaml_data()
        ugv_data = yaml_data['marsupial_ugv']
        waypoint_list = []
        poses_keys = sorted([k for k in ugv_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
        for key in poses_keys:
            pos = ugv_data[key]['pose']['position']
            waypoint_list.append((pos['x'], pos['y'], self.ugv_initial_height))
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
    def compute_lookahead_distance(self, waypoints, initial_wp, is_uav):
        total_distance_xy = 0.0
        total_distance_z = 0.0

        wp = waypoints[initial_wp]

        if self.uav_tol is not None and self.ugv_tol is not None:
            # Distancia entre la posición actual hasta el waypoint inicial
            if is_uav:
                dx = wp[0] - self.current_uav_pose.x
                dy = wp[1] - self.current_uav_pose.y
                dz = wp[2] - self.current_uav_pose.z
                total_distance_xy += math.sqrt(dx**2 + dy**2)
                total_distance_z += abs(dz)
            elif not is_uav:
                dx = wp[0] - self.current_ugv_pose.x
                dy = wp[1] - self.current_ugv_pose.y
                total_distance_xy += math.sqrt(dx**2 + dy**2)

            # Ahora sumamos las distancias entre los siguientes waypoints
            for i in range(initial_wp, min(self.final_wp, len(waypoints)-1)):
                wp_current = waypoints[i]
                wp_next = waypoints[i+1]

                    # Si es UAV (3D)
                if is_uav:
                    dx = wp_next[0] - wp_current[0]
                    dy = wp_next[1] - wp_current[1]
                    dz = wp_next[2] - wp_current[2]
                    total_distance_xy += math.sqrt(dx**2 + dy**2)
                    total_distance_z += abs(dz)

                    # Si es UGV (2D)
                elif not is_uav:
                    dx = wp_next[0] - wp_current[0]
                    dy = wp_next[1] - wp_current[1]
                    total_distance_xy += math.sqrt(dx**2 + dy**2)

            # if is_uav:
            #     total_distance_xy = total_distance_xy - self.uav_tol
            #     total_distance_z = total_distance_z - self.uav_tol
            # else:
            #     total_distance_xy = total_distance_xy - self.ugv_tol

            # if total_distance_xy < 0:
            #     total_distance_xy = 0.0
            # if total_distance_z < 0:
            #     total_distance_z = 0.0

            return total_distance_xy, total_distance_z

    # -------------------- COMPROBACIONES DE LLEGADAS --------------------
    def check_ugv_reached(self):
        if self.current_ugv_wp >= self.final_wp:
            self.ugv_reached = True
            return True

    def check_uav_reached(self):
        if self.current_uav_wp >= self.final_wp:
            self.uav_reached = True
            return True

    def debug_callback(self):
        # if(self.current_uav_wp is not None and self.current_ugv_wp):
        #     self.get_logger().info("------ ESTADO ACTUAL ------")
        #     self.get_logger().info(f"WP actual de UAV: {self.current_uav_wp}")
        #     self.get_logger().info(f"WP actual de UGV: {self.current_ugv_wp}")
        #     self.get_logger().info(f"ERROR: {abs(self.current_uav_wp-self.current_ugv_wp)}")
        #     self.get_logger().info("-----------------------------")
        return

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
            self.v_uav_xy = self.v_min
            self.v_uav_z = self.v_min
            rclpy.shutdown()
            return

        self.initial_wp_uav = self.current_uav_wp
        self.initial_wp_ugv = self.current_ugv_wp

        # Calcular distancias de lookahead
        D_ugv, _ = self.compute_lookahead_distance(self.waypoints_ugv, self.initial_wp_ugv, is_uav=False)
        D_uav_xy, D_uav_z = self.compute_lookahead_distance(self.waypoints_uav, self.initial_wp_uav, is_uav=True)


        # Parámetros de velocidades de cada vehículo
        v_min = self.v_min
        v_max_ugv = self.v_max_ugv
        v_max_uav = self.v_max_uav

        # Tiempo objetivo usando v_max para el más lejano
        t_ugv = D_ugv / v_max_ugv 
        t_uav = D_uav_xy / v_max_uav 

        t_objetivo = max(t_ugv, t_uav)

        # Calcular velocidad para el otro vehículo
        v_ugv = D_ugv / t_objetivo
        v_uav = D_uav_xy / t_objetivo

        # Limitar velocidad para que esté dentro de [v_min, v_max] con v_max particular para cada vehículo
        v_ugv = min(max(v_ugv, v_min), v_max_ugv)
        v_uav = min(max(v_uav, v_min), v_max_uav)

        tol_uav_z = 0.1  # Tolerancia para la altura del UAV

        # Asignar velocidades
        self.v_ugv = v_ugv
        self.v_uav_xy = v_uav
        self.v_uav_z = v_uav

        objetivo_z = self.waypoints_uav[self.current_uav_wp]

        if self.current_uav_pose.z > objetivo_z[2] - tol_uav_z and self.current_uav_pose.z < objetivo_z[2] + tol_uav_z:
            self.v_uav_z = v_min

        #self.get_logger().info(f"t_objetivo: {t_objetivo:.2f} s")

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
