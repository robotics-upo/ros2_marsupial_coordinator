#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance
import yaml
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

def load_yaml_data():
        yaml_filename = 'optimized_path_teatro_wall_opt.yaml'

        package_share_directory = get_package_share_directory('ros2_marsupial_coordinator')
        yaml_file_path = os.path.join(package_share_directory, 'config', yaml_filename)

        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        return yaml_data

def load_waypoints_length():
    yaml_data = load_yaml_data()
    tether_data = yaml_data['tether']
    waypoint_list = []
    poses_keys = sorted([k for k in tether_data.keys() if k.startswith("length")], key=lambda k: int(k[6:]))
    for key in poses_keys:
        l = tether_data[key]['length']
        waypoint_list.append(l)
    return waypoint_list

def load_waypoints_UGV():
    yaml_data = load_yaml_data()
    ugv_data = yaml_data['marsupial_ugv']
    waypoint_list = []
    poses_keys = sorted([k for k in ugv_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
    for key in poses_keys:
        pos = ugv_data[key]['pose']['position']
        waypoint_list.append((pos['x'], pos['y']))
    return waypoint_list

def load_waypoints_UAV():
    yaml_data = load_yaml_data()
    uav_data = yaml_data['marsupial_uav']
    waypoint_list = []
    poses_keys = sorted([k for k in uav_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
    for key in poses_keys:
        pos = uav_data[key]['pose']['position']
        waypoint_list.append((pos['x'], pos['y'], pos['z']))
    return waypoint_list

# --- Función para cargar datos CSV ---
def load_csv(filename):
    return pd.read_csv(filename)

def plot_uav_trajectory_2D(uav_pos, waypoints_uav, uav_speed_xy, uav_speed_z):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Trayectoria del UAV en 2D", fontsize=16)
    
    # --- Subplot 1: 
    axs[0].plot(uav_pos['position.x'], uav_pos['position.y'], label="Posición UAV")
    axs[0].scatter(waypoints_uav[:, 0], waypoints_uav[:, 1], color='r', marker='x', label="Waypoints UAV")
    axs[0].set_title("Vista superior (Plano XY)")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].legend()

    # --- Subplot 2: 
    axs[1].plot(uav_pos['position.y'], uav_pos['position.z'], label="Posición UAV")
    axs[1].scatter(waypoints_uav[:, 1], waypoints_uav[:, 2], color='r', marker='x', label="Waypoints UAV")
    axs[1].set_title("Vista lateral (Plano YZ)")
    axs[1].set_xlabel("Y [m]")
    axs[1].set_ylabel("Z [m]")
    axs[1].axis("equal")
    axs[1].grid(True)
    axs[1].legend()
    
    plt.tight_layout()

def plot_uav_trajectory_3D(uav_pos, waypoints_uav):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')  # Vista 3D

    # Trayectoria del UAV
    ax.plot(uav_pos['position.x'], uav_pos['position.y'], uav_pos['position.z'],
            label='Trayectoria UAV', color='blue')

    # Waypoints
    ax.scatter(waypoints_uav[:, 0], waypoints_uav[:, 1], waypoints_uav[:, 2],
               color='red', marker='x', s=50, label='Waypoints')

    # Etiquetas y leyenda
    ax.set_title("Trayectoria 3D del UAV", fontsize=16)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    ax.grid(True)

    # Ajuste de aspecto
    ax.set_box_aspect([1, 1, 1])  # Hace que los ejes tengan proporción 1:1:1
    plt.tight_layout()

def plot_uav_error(uav_pos, waypoints_uav, uav_speed_xy, uav_speed_z):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Error cometido en posición", fontsize=16)
    
    # Graficar en subplot 1: eje x = tiempo, eje y = waypoint indice (ref, y) - uavpospositiony
    # Graficar en subplot 2: eje x = tiempo, eje y = waypoint indice (ref, x) - uavpospositionx
    # Graficar en subplot 3: eje x = tiempo, eje y = waypoint indice (ref, z) - uavpospositionz

    # --- Subplot 1: 
    axs[0].plot(uav_pos['time'], waypoints_uav[:, 0] - uav_pos['position.x'], label="Posición UAV")
    axs[0].scatter(waypoints_uav[:, 0], waypoints_uav[:, 1], color='r', marker='x', label="Waypoints UAV")
    axs[0].set_title("Vista superior (Plano XY)")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].legend()

    # --- Subplot 2: 
    axs[1].plot(uav_pos['position.y'], uav_pos['position.z'], label="Posición UAV")
    axs[1].scatter(waypoints_uav[:, 1], waypoints_uav[:, 2], color='r', marker='x', label="Waypoints UAV")
    axs[1].set_title("Vista lateral (Plano YZ)")
    axs[1].set_xlabel("Y [m]")
    axs[1].set_ylabel("Z [m]")
    axs[1].axis("equal")
    axs[1].grid(True)
    axs[1].legend()
    
    plt.tight_layout()

# --- MAIN ---
def main():


    waypoints_uav = load_waypoints_UAV()
    waypoints_ugv = load_waypoints_UGV()
    waypoints_length = load_waypoints_length()

    waypoints_uav = np.array(load_waypoints_UAV())
    waypoints_ugv = np.array(load_waypoints_UGV())
    waypoints_length = np.array(load_waypoints_length())   

    # Cargar datos
    uav_pos = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/sjtu_drone_gt_pose.csv")
    ugv_pos = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/rs_robot_ugv_gt_pose.csv")
    uav_speed_xy = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_reference_speed_xy.csv")
    uav_speed_z = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_reference_speed_z.csv")
    ugv_speed = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/ugv_reference_speed.csv")
    cable_length = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cable_length.csv")
    winch_velocity = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/forward_velocity_controller_commands.csv")

    # Gráficas
    plot_uav_trajectory_2D(uav_pos, waypoints_uav, uav_speed_xy, uav_speed_z)
    plot_uav_trajectory_3D(uav_pos, waypoints_uav)

    # plot_ugv_control(ugv_speed)
    # plot_cable(cable_length, winch_velocity)

    # plot_uav_trajectory(uav_pos, waypoints_uav)
    # plot_ugv_trajectory(ugv_pos, waypoints_ugv)

    # # Cálculo de errores (usamos solo x,y,z del UAV)
    # real_uav_xyz = uav_pos[['x', 'y', 'z']].values
    # plot_tracking_error(real_uav_xyz, waypoints_uav, name="UAV")

    # # Cálculo de errores para el UGV (x,y)
    # real_ugv_xy = ugv_pos[['x', 'y']].values
    # plot_tracking_error(real_ugv_xy, waypoints_ugv, name="UGV")

    # Mostrar todo
    plt.show()

if __name__ == "__main__":
    main()
