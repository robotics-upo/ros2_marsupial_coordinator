#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance
import yaml
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
from scipy.interpolate import interp1d
from matplotlib.ticker import MultipleLocator

def load_yaml_data():
        yaml_filename = 'optimized_path_teatro_wall_opt_r.yaml'
        #yaml_filename = 'teatro_mission.yaml'

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
    ugv_initial_height = 0.274931
    yaml_data = load_yaml_data()
    ugv_data = yaml_data['marsupial_ugv']
    waypoint_list = []
    poses_keys = sorted([k for k in ugv_data.keys() if k.startswith("poses")], key=lambda k: int(k[5:]))
    for key in poses_keys:
        pos = ugv_data[key]['pose']['position']
        waypoint_list.append((pos['x'], pos['y'], ugv_initial_height))
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

def load_csv(filename):
    return pd.read_csv(filename)

# === GRÁFICAS PARA EL UAV ===
def plot_uav_trajectory_2D(uav_pos, waypoints_uav):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Trayectoria del UAV en 2D", fontsize=16)

    x_init = uav_pos['position.x'].iloc[0]
    y_init = uav_pos['position.y'].iloc[0]
    z_init = uav_pos['position.z'].iloc[0]

    x_final = uav_pos['position.x'].iloc[-1]
    y_final = uav_pos['position.y'].iloc[-1]
    z_final = uav_pos['position.z'].iloc[-1]

    
    # Subplot 1: 
    axs[0].plot(uav_pos['position.x'], uav_pos['position.y'], label="Posición UAV")
    axs[0].scatter(waypoints_uav[:, 0], waypoints_uav[:, 1], color='r', marker='x', label="Waypoints UAV")
    axs[0].scatter(x_init, y_init, color='green', marker='o', label="Inicio", zorder=5)
    axs[0].scatter(x_final, y_final, color='k', marker='o', label="Fin", zorder=5)
    axs[0].set_title("Vista superior (Plano XY)")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    #axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: 
    axs[1].plot(uav_pos['position.x'], uav_pos['position.z'], label="Posición UAV")
    axs[1].scatter(waypoints_uav[:, 0], waypoints_uav[:, 2], color='r', marker='x', label="Waypoints UAV")
    axs[1].scatter(x_init, z_init, color='green', marker='o', label="Inicio", zorder=5)
    axs[1].scatter(x_final, z_final, color='k', marker='o', label="Fin", zorder=5)
    axs[1].set_title("Vista lateral (Plano XZ)")
    axs[1].set_xlabel("X [m]")
    axs[1].set_ylabel("Z [m]")
    #axs[1].axis("equal")
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
    
    x_init = uav_pos['position.x'].iloc[0]
    y_init = uav_pos['position.y'].iloc[0]
    z_init = uav_pos['position.z'].iloc[0]

    x_final = uav_pos['position.x'].iloc[-1]
    y_final = uav_pos['position.y'].iloc[-1]
    z_final = uav_pos['position.z'].iloc[-1]


    ax.scatter(x_init, y_init, z_init,
               color='green', marker='o', s=100, label='Inicio', zorder=5)


    ax.scatter(x_final, y_final, z_final,
               color='k', marker='o', s=100, label='Fin', zorder=5)

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

def plot_uav_error(uav_pos, waypoints_uav, reference_index_uav):

    x_init = uav_pos['position.x'].iloc[0]
    y_init = uav_pos['position.y'].iloc[0]
    z_init = uav_pos['position.z'].iloc[0]

    x_final = uav_pos['position.x'].iloc[-1]
    y_final = uav_pos['position.y'].iloc[-1]
    z_final = uav_pos['position.z'].iloc[-1]


    uav_pos['time'] = uav_pos['time'] / 1e9
    reference_index_uav['time'] = reference_index_uav['time'] / 1e9

    interp_indices = interp1d(reference_index_uav['time'], reference_index_uav['data'], kind='previous', bounds_error=False, fill_value='extrapolate')

    ref_indices = interp_indices(uav_pos['time'])

    # Aseguramos que los índices son enteros válidos
    ref_indices = np.clip(ref_indices.astype(int), 0, waypoints_uav.shape[0] - 1)

    # Obtenemos la posición X de los waypoints en esos instantes
    waypoints_x = waypoints_uav[ref_indices, 0]
    waypoints_y = waypoints_uav[ref_indices, 1]
    waypoints_z = waypoints_uav[ref_indices, 2]

    # Calculamos el error
    error_x = waypoints_x - uav_pos['position.x']
    error_y = waypoints_y - uav_pos['position.y']
    error_z = waypoints_z - uav_pos['position.z']

    fig, axs = plt.subplots(3, 2, figsize=(10, 8))  # filas, columna

    fig.suptitle("Error y trayectoria del UAV a lo largo del tiempo", fontsize=16)

    # Subplots columna izquierda (1-3)
    #axs[0, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 0].set_title("Error cometido en cada eje")
    axs[0, 0].plot(uav_pos['time'], error_x, label="Error en X", color='r')
    axs[0, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[0, 0].axhline(y=0.4, color='y', linestyle='--', label="Lookahead distance")
    axs[0, 0].axhline(y=-0.4, color='y', linestyle='--')
    axs[0, 0].set_xlabel("t [s]")
    axs[0, 0].set_ylabel("error X [m]")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    #axs[1, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 0].plot(uav_pos['time'], error_y, label="Error en Y", color='g')
    axs[1, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[1, 0].axhline(y=0.4, color='y', linestyle='--', label="Lookahead distance")
    axs[1, 0].axhline(y=-0.4, color='y', linestyle='--')
    axs[1, 0].set_xlabel("t [s]")
    axs[1, 0].set_ylabel("error Y [m]")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    #axs[2, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[2, 0].plot(uav_pos['time'], error_z, label="Error en Z", color='b')
    axs[2, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[2, 0].axhline(y=0.4, color='y', linestyle='--', label="Lookahead distance")
    axs[2, 0].axhline(y=-0.4, color='y', linestyle='--')
    axs[2, 0].set_xlabel("t [s]")
    axs[2, 0].set_ylabel("error Z [m]")
    axs[2, 0].grid(True)
    axs[2, 0].legend()

    # Subplots columna derecha (4-6)
    #axs[0, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 1].set_title("Trayectoria y referencia en cada eje")
    axs[0, 1].plot(uav_pos['time'], uav_pos['position.x'], label="Trayectoria en X", color='r')
    axs[0, 1].plot(uav_pos['time'], waypoints_x, label="Referencia en X", color='k')
    axs[0, 1].set_xlabel("t [s]")
    axs[0, 1].set_ylabel("X [m]")
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    #axs[1, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 1].plot(uav_pos['time'], uav_pos['position.y'], label="Trayectoria en Y", color='g')
    axs[1, 1].plot(uav_pos['time'], waypoints_y, label="Referencia en Y", color='k')
    axs[1, 1].set_xlabel("t [s]")
    axs[1, 1].set_ylabel("Y [m]")
    axs[1, 1].grid(True)
    axs[1, 1].legend()

    #axs[2, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[2, 1].plot(uav_pos['time'], uav_pos['position.z'], label="Trayectoria en Z", color='b')
    axs[2, 1].plot(uav_pos['time'], waypoints_z, label="Referencia en Z", color='k')
    axs[2, 1].set_xlabel("t [s]")
    axs[2, 1].set_ylabel("Z [m]")
    axs[2, 1].grid(True)
    axs[2, 1].legend()
    
    plt.tight_layout()

def plot_uav_error_adaptative(uav_pos, waypoints_uav, reference_index_uav, lh_uav):

    x_init = uav_pos['position.x'].iloc[0]
    y_init = uav_pos['position.y'].iloc[0]
    z_init = uav_pos['position.z'].iloc[0]

    x_final = uav_pos['position.x'].iloc[-1]
    y_final = uav_pos['position.y'].iloc[-1]
    z_final = uav_pos['position.z'].iloc[-1]

    lh_uav['time'] = lh_uav['time'] / 1e9
    uav_pos['time'] = uav_pos['time'] / 1e9
    reference_index_uav['time'] = reference_index_uav['time'] / 1e9

    interp_indices = interp1d(reference_index_uav['time'], reference_index_uav['data'], kind='previous', bounds_error=False, fill_value='extrapolate')

    ref_indices = interp_indices(uav_pos['time'])

    # Aseguramos que los índices son enteros válidos
    ref_indices = np.clip(ref_indices.astype(int), 0, waypoints_uav.shape[0] - 1)

    # Obtenemos la posición X de los waypoints en esos instantes
    waypoints_x = waypoints_uav[ref_indices, 0]
    waypoints_y = waypoints_uav[ref_indices, 1]
    waypoints_z = waypoints_uav[ref_indices, 2]

    # Calculamos el error
    error_x = waypoints_x - uav_pos['position.x']
    error_y = waypoints_y - uav_pos['position.y']
    error_z = waypoints_z - uav_pos['position.z']

    fig, axs = plt.subplots(3, 2, figsize=(10, 8))  # filas, columna

    fig.suptitle("Error y trayectoria del UAV a lo largo del tiempo", fontsize=16)

    # Subplots columna izquierda (1-3)
    #axs[0, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 0].set_title("Error cometido en cada eje")
    axs[0, 0].plot(uav_pos['time'], error_x, label="Error en X", color='r')
    axs[0, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[0, 0].plot(lh_uav['time'], lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[0, 0].plot(lh_uav['time'], -lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[0, 0].set_xlabel("t [s]")
    axs[0, 0].set_ylabel("error X [m]")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    #axs[1, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 0].plot(uav_pos['time'], error_y, label="Error en Y", color='g')
    axs[1, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[1, 0].plot(lh_uav['time'], lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[1, 0].plot(lh_uav['time'], -lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[1, 0].set_xlabel("t [s]")
    axs[1, 0].set_ylabel("error Y [m]")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    #axs[2, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[2, 0].plot(uav_pos['time'], error_z, label="Error en Z", color='b')
    axs[2, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[2, 0].plot(lh_uav['time'], lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[2, 0].plot(lh_uav['time'], -lh_uav['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[2, 0].set_xlabel("t [s]")
    axs[2, 0].set_ylabel("error Z [m]")
    axs[2, 0].grid(True)
    axs[2, 0].legend()

    # Subplots columna derecha (4-6)
    #axs[0, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 1].set_title("Trayectoria y referencia en cada eje")
    axs[0, 1].plot(uav_pos['time'], uav_pos['position.x'], label="Trayectoria en X", color='r')
    axs[0, 1].plot(uav_pos['time'], waypoints_x, label="Referencia en X", color='k')
    axs[0, 1].set_xlabel("t [s]")
    axs[0, 1].set_ylabel("X [m]")
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    #axs[1, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 1].plot(uav_pos['time'], uav_pos['position.y'], label="Trayectoria en Y", color='g')
    axs[1, 1].plot(uav_pos['time'], waypoints_y, label="Referencia en Y", color='k')
    axs[1, 1].set_xlabel("t [s]")
    axs[1, 1].set_ylabel("Y [m]")
    axs[1, 1].grid(True)
    axs[1, 1].legend()

    #axs[2, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[2, 1].plot(uav_pos['time'], uav_pos['position.z'], label="Trayectoria en Z", color='b')
    axs[2, 1].plot(uav_pos['time'], waypoints_z, label="Referencia en Z", color='k')
    axs[2, 1].set_xlabel("t [s]")
    axs[2, 1].set_ylabel("Z [m]")
    axs[2, 1].grid(True)
    axs[2, 1].legend()
    
    plt.tight_layout()

    #ax.set_xlim([20, 80])

# === GRÁFICAS PARA EL UGV ===
def plot_ugv_trajectory_2D(ugv_pos, waypoints_ugv):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    x_init = ugv_pos['position.x'].iloc[0]
    y_init = ugv_pos['position.y'].iloc[0]
    z_init = ugv_pos['position.z'].iloc[0]

    x_final = ugv_pos['position.x'].iloc[-1]
    y_final = ugv_pos['position.y'].iloc[-1]
    z_final = ugv_pos['position.z'].iloc[-1]

    fig.suptitle("Trayectoria del UGV en 2D", fontsize=16)
    
    # Subplot 1: 
    axs[0].plot(ugv_pos['position.x'], ugv_pos['position.y'], label="Posición UGV")
    axs[0].scatter(waypoints_ugv[:, 0], waypoints_ugv[:, 1], color='r', marker='x', label="Waypoints UGV")
    axs[0].scatter(x_init, y_init, color='green', marker='o', label="Inicio", zorder=5)
    axs[0].scatter(x_final, y_final, color='k', marker='o', label="Fin", zorder=5)
    axs[0].set_title("Vista superior (Plano XY)")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: 
    axs[1].plot(ugv_pos['position.x'], ugv_pos['position.z'], label="Posición UGV")
    axs[1].scatter(waypoints_ugv[:, 0], waypoints_ugv[:, 2], color='r', marker='x', label="Waypoints UGV")
    axs[1].scatter(x_init, z_init, color='green', marker='o', label="Inicio", zorder=5)
    axs[1].scatter(x_final, z_final, color='k', marker='o', label="Fin", zorder=5)
    axs[1].set_title("Vista lateral (Plano XZ)")
    axs[1].set_xlabel("X [m]")
    axs[1].set_ylabel("Z [m]")
    axs[1].grid(True)
    axs[1].legend()
    axs[1].set_yticks(np.arange(0, 1, 0.1))

    # Sincronizar el eje X en ambos subplots (usando position.x)
    x_vals = ugv_pos['position.x']
    x_min = x_vals.min()
    x_max = x_vals.max()

    axs[0].set_xlim(x_min-1, x_max+1)  # Subplot XY
    axs[1].set_xlim(x_min-1, x_max+1)  # Subplot XZ


    
    plt.tight_layout()

def plot_ugv_trajectory_3D(ugv_pos, waypoints_ugv):
    fig = plt.figure(figsize=(10, 7))

    x_init = ugv_pos['position.x'].iloc[0]
    y_init = ugv_pos['position.y'].iloc[0]
    z_init = ugv_pos['position.z'].iloc[0]

    x_final = ugv_pos['position.x'].iloc[-1]
    y_final = ugv_pos['position.y'].iloc[-1]
    z_final = ugv_pos['position.z'].iloc[-1]

    ax = fig.add_subplot(111, projection='3d')  

    ax.plot(ugv_pos['position.x'], ugv_pos['position.y'], ugv_pos['position.z'],
            label='Trayectoria UGV', color='blue')

    # Waypoints
    ax.scatter(waypoints_ugv[:, 0], waypoints_ugv[:, 1], waypoints_ugv[:, 2],
               color='red', marker='x', s=50, label='Waypoints')
    
    ax.scatter(x_init, y_init, z_final, color='green', marker='o', label="Inicio", zorder=5)
    ax.scatter(x_final, y_final, z_final, color='k', marker='o', label="Fin", zorder=5)
    
    ax.set_zticks(np.arange(0, 1, 0.1)) # eje z de 0 a 1 metro con pasos de 0.1

    # Etiquetas y leyenda
    ax.set_title("Trayectoria 3D del UGV", fontsize=16)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    ax.grid(True)

    # Ajuste de aspecto
    ax.set_box_aspect([1, 1, 1])  # Hace que los ejes tengan proporción 1:1:1
    plt.tight_layout()

def plot_ugv_error(ugv_pos, waypoints_ugv, reference_index_ugv):

    x_init = ugv_pos['position.x'].iloc[0]
    y_init = ugv_pos['position.y'].iloc[0]
    z_init = ugv_pos['position.z'].iloc[0]

    x_final = ugv_pos['position.x'].iloc[-1]
    y_final = ugv_pos['position.y'].iloc[-1]
    z_final = ugv_pos['position.z'].iloc[-1]

    ugv_pos['time'] = ugv_pos['time'] / 1e9
    reference_index_ugv['time'] = reference_index_ugv['time'] / 1e9

    interp_indices = interp1d(reference_index_ugv['time'], reference_index_ugv['data'], kind='previous', bounds_error=False, fill_value='extrapolate')

    ref_indices = interp_indices(ugv_pos['time'])

    # Aseguramos que los índices son enteros válidos
    ref_indices = np.clip(ref_indices.astype(int), 0, waypoints_ugv.shape[0] - 1)

    # Obtenemos la posición X de los waypoints en esos instantes
    waypoints_x = waypoints_ugv[ref_indices, 0]
    waypoints_y = waypoints_ugv[ref_indices, 1]

    # Calculamos el error
    error_x = waypoints_x - ugv_pos['position.x']
    error_y = waypoints_y - ugv_pos['position.y']

    fig, axs = plt.subplots(2, 2, figsize=(10, 8))  # filas, columna

    fig.suptitle("Error cometido en posición por el UGV", fontsize=16)

    # Subplots columna izquierda (1-2)
    #axs[0, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 0].plot(ugv_pos['time'], error_x, label="Error en X", color='r')
    axs[0, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[0, 0].axhline(y=0.4, color='y', linestyle='--')
    axs[0, 0].axhline(y=-0.4, color='y', linestyle='--', label="Lookahead distance")
    axs[0, 0].set_xlabel("t [s]")
    axs[0, 0].set_ylabel("error X [m]")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    #axs[1, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 0].plot(ugv_pos['time'], error_y, label="Error en Y", color='g')
    axs[1, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[1, 0].axhline(y=0.4, color='y', linestyle='--')
    axs[1, 0].axhline(y=-0.4, color='y', linestyle='--', label="Lookahead distance")
    axs[1, 0].set_xlabel("t [s]")
    axs[1, 0].set_ylabel("error Y [m]")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    # Subplots columna derecha (3-4)
    #axs[0, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 1].plot(ugv_pos['time'], ugv_pos['position.x'], label="Trayectoria en X", color='r')
    axs[0, 1].plot(ugv_pos['time'], waypoints_x, label="Referencia en X", color='k')
    axs[0, 1].set_xlabel("t [s]")
    axs[0, 1].set_ylabel("X [m]")
    axs[0, 1].grid(True)
    axs[0, 1].legend()  

    #axs[1, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 1].plot(ugv_pos['time'], ugv_pos['position.y'], label="Trayectoria en Y", color='g')
    axs[1, 1].plot(ugv_pos['time'], waypoints_y, label="Referencia en Y", color='k')
    axs[1, 1].set_xlabel("t [s]")
    axs[1, 1].set_ylabel("Y [m]")          
    axs[1, 1].grid(True)
    axs[1, 1].legend()
    plt.tight_layout()

def plot_ugv_error_adaptative(ugv_pos, waypoints_ugv, reference_index_ugv, lh_ugv):
    
    x_init = ugv_pos['position.x'].iloc[0]
    y_init = ugv_pos['position.y'].iloc[0]
    z_init = ugv_pos['position.z'].iloc[0]

    x_final = ugv_pos['position.x'].iloc[-1]
    y_final = ugv_pos['position.y'].iloc[-1]
    z_final = ugv_pos['position.z'].iloc[-1]

    lh_ugv['time'] = lh_ugv['time'] / 1e9
    ugv_pos['time'] = ugv_pos['time'] / 1e9
    reference_index_ugv['time'] = reference_index_ugv['time'] / 1e9

    interp_indices = interp1d(reference_index_ugv['time'], reference_index_ugv['data'], kind='previous', bounds_error=False, fill_value='extrapolate')

    ref_indices = interp_indices(ugv_pos['time'])

    # Aseguramos que los índices son enteros válidos
    ref_indices = np.clip(ref_indices.astype(int), 0, waypoints_ugv.shape[0] - 1)

    # Obtenemos la posición X de los waypoints en esos instantes
    waypoints_x = waypoints_ugv[ref_indices, 0]
    waypoints_y = waypoints_ugv[ref_indices, 1]

    # Calculamos el error
    error_x = waypoints_x - ugv_pos['position.x']
    error_y = waypoints_y - ugv_pos['position.y']

    fig, axs = plt.subplots(2, 2, figsize=(10, 8))  # filas, columna

    fig.suptitle("Error cometido en posición por el UGV", fontsize=16)

    # Subplots columna izquierda (1-2)
    #axs[0, 0].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 0].set_title("Error cometido en cada eje")
    axs[0, 0].plot(ugv_pos['time'], error_x, label="Error en X", color='r')
    axs[0, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[0, 0].plot(lh_ugv['time'], lh_ugv['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[0, 0].plot(lh_ugv['time'], -lh_ugv['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[0, 0].set_xlabel("t [s]")   
    axs[0, 0].set_ylabel("error X [m]")
    axs[0, 0].grid(True)
    axs[0, 0].legend()
    
    axs[1, 0].plot(ugv_pos['time'], error_y, label="Error en Y", color='g')
    axs[1, 0].axhline(y=0.0, color='k', linestyle='--')
    axs[1, 0].plot(lh_ugv['time'], lh_ugv['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[1, 0].plot(lh_ugv['time'], -lh_ugv['data'], label="Lookahead distance", color='y', linestyle='--')
    axs[1, 0].set_xlabel("t [s]")
    axs[1, 0].set_ylabel("error Y [m]")
    axs[1, 0].grid(True)
    axs[1, 0].legend()  

    # Subplots columna derecha (3-4)
    #axs[0, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[0, 1].set_title("Trayectoria y referencia en cada eje")
    axs[0, 1].plot(ugv_pos['time'], ugv_pos['position.x'], label="Trayectoria en X", color='r')
    axs[0, 1].plot(ugv_pos['time'], waypoints_x, label="Referencia en X", color='k')
    axs[0, 1].set_xlabel("t [s]")
    axs[0, 1].set_ylabel("X [m]")
    axs[0, 1].grid(True)
    axs[0, 1].legend() 

    #axs[1, 1].xaxis.set_major_locator(MultipleLocator(5))
    axs[1, 1].plot(ugv_pos['time'], ugv_pos['position.y'], label="Trayectoria en Y", color='g')
    axs[1, 1].plot(ugv_pos['time'], waypoints_y, label="Referencia en Y ", color='k')
    axs[1, 1].set_xlabel("t [s]")
    axs[1, 1].set_ylabel("Y [m]")
    axs[1, 1].grid(True)
    axs[1, 1].legend()  
    plt.tight_layout()




# === GRÁFICAS PARA EL CABLE ===
def plot_tether_trajectory(cable_length, winch_velocity, cable_length_1, cable_length_2):
    cable_length['time'] = cable_length['time'] / 1e9  # Convertir tiempo a segundos
    winch_velocity['time'] = winch_velocity['time'] / 1e9  

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Cable (tether)", fontsize=16)

    # Subplot 1: Longitud del cable
    axs[0].plot(cable_length_2['time']/1e9, cable_length_2['data[0]'], label="Longitud kp=1", color='b')
    axs[0].plot(cable_length_2['time']/1e9, cable_length_2['data[1]'], label="Longitud de referencia kp=1", color='k', linestyle='--')
    #axs[0].plot(cable_length['time'], cable_length['data[2]'], label="Longitud de referencia kp=1", color='k', linestyle='--')
    axs[0].set_title("Longitud del cable a lo largo del tiempo")
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("Longitud [m]")
    axs[0].grid(True)
    axs[0].legend()

   # Subplot 2: Error de longitud del cable
    # axs[1].plot(cable_length['time'], cable_length['data[1]'] - cable_length['data[0]'], label="Error cometido", color='r')
    # axs[1].axhline(y=0.0, color='k', linestyle='--')
    # axs[1].set_title("Error")
    # axs[1].set_xlabel("t [s]")
    # axs[1].set_ylabel("error [m]")
    # axs[1].grid(True)
    # axs[1].legend()

    # Subplot 3: Señal de control aplicada, velocidad del winch
    axs[1].plot(winch_velocity['time'], winch_velocity['data[4]'], label="Velocidad angular del cabrestante", color='g')
    axs[1].set_title("Velocidad del cabrestante a lo largo del tiempo")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("w [rad/s]")
    axs[1].grid(True)
    axs[1].legend()


    plt.tight_layout()

# == GRÁFICAS PARA VER DESEMPEÑO DE TRAYECTORIA COORDINADA ==
def plot_coordinator_performance(reference_index_ugv, reference_index_uav):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Desempeño del coordinador", fontsize=16)

    # Subplot 1: Trayectoria UAV vs UGV
    axs[0].xaxis.set_major_locator(MultipleLocator(20))
    axs[0].plot(reference_index_ugv['time'], reference_index_ugv['data'], label="Índice UGV", color='r')
    axs[0].plot(reference_index_uav['time'], reference_index_uav['data'], label="Índice UAV", color='b')
    axs[0].set_title("Índices de referencia de UGV y UAV en el tiempo")
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("Índice")
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 1: Error
    axs[1].xaxis.set_major_locator(MultipleLocator(20))
    axs[1].plot(reference_index_ugv['time'], reference_index_uav['data'] - reference_index_ugv['data'], label="Índice UGV", color='r')
    axs[1].axhline(y=0.0, color='k', linestyle='--')
    axs[1].set_title("Error de índices UAV-UGV")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("error de índice")
    axs[1].grid(True)
    axs[1].legend()

    plt.tight_layout()

# == GRÁFICAS DE VELOCIDADES ==
def plot_speeds(ugv_speed, uav_speed_xy, uav_speed_z):
    ugv_speed['time'] = ugv_speed['time'] / 1e9  # Convertir tiempo a segundos
    uav_speed_xy['time'] = uav_speed_xy['time'] / 1e9  
    uav_speed_z['time'] = uav_speed_z['time'] / 1e9 

    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Velocidades enviadas por el coordinador", fontsize=16)

    # Subplot 2: Señal de control aplicada al UGV
    #axs[0].xaxis.set_major_locator(MultipleLocator(20))
    axs[0].plot(ugv_speed['time'], ugv_speed['data'], label="Velocidad UGV", color='r')
    axs[0].set_title("Velocidad del UGV a lo largo del tiempo")
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("v [m/s]")
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 3: Velocidad del UAV en el eje X e Y
    #axs[1].xaxis.set_major_locator(MultipleLocator(20))
    axs[1].plot(uav_speed_xy['time'], uav_speed_xy['data'], label="Velocidad UAV en XY", color='y')
    axs[1].set_title("Velocidad del UAV en XY a lo largo del tiempo")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("v [m/s]")
    axs[1].grid(True)
    axs[1].legend()

    # Subplot 4: Velocidad del UAV en el eje Z
    #axs[2].xaxis.set_major_locator(MultipleLocator(20))
    axs[2].plot(uav_speed_z['time'], uav_speed_z['data'], label="Velocidad UAV en Z", color='b')
    axs[2].set_title("Velocidad del UAV en Z a lo largo del tiempo")
    axs[2].set_xlabel("t [s]")
    axs[2].set_ylabel("v [m/s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()

def plot_coordinator_comparison(reference_index_ugv, reference_index_uav, reference_index_ugv_1, reference_index_uav_1, reference_index_uav_2, reference_index_ugv_2, reference_index_ugv_3, reference_index_uav_3):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # filas, columna

    fig.suptitle("Coomparación coordinación con diferentes valores de ventana N", fontsize=16)

    # Subplot 1: Trayectoria UAV vs UGV
    axs[0].xaxis.set_major_locator(MultipleLocator(20))
    axs[0].plot(reference_index_ugv['time'], reference_index_ugv['data'], label="Índice UGV, N=0", color='r')
    axs[0].plot(reference_index_uav['time'], reference_index_uav['data'], label="Índice UAV, N=0", color='b')
    axs[0].plot(reference_index_ugv_1['time']/1e9, reference_index_ugv_1['data'], label="Índice UGV, N=5", color='g')
    axs[0].plot(reference_index_uav_1['time']/1e9, reference_index_uav_1['data'], label="Índice UAV, N=5", color='y')
    axs[0].plot(reference_index_ugv_2['time']/1e9, reference_index_ugv_2['data'], label="Índice UGV, N=15", color='c')
    axs[0].plot(reference_index_uav_2['time']/1e9, reference_index_uav_2['data'], label="Índice UAV, N=15", color='m')
    axs[0].plot(reference_index_ugv_3['time']/1e9, reference_index_ugv_3['data'], label="Índice UGV, N=45", color='k')
    axs[0].plot(reference_index_uav_3['time']/1e9, reference_index_uav_3['data'], label="Índice UAV, N=45", color='orange')
    axs[0].set_title("Índices de referencia de UGV y UAV en el tiempo")
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("Índice")
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 1: Error
    axs[1].xaxis.set_major_locator(MultipleLocator(20))
    axs[1].plot(reference_index_ugv['time'], reference_index_uav['data'] - reference_index_ugv['data'], label="Error para coordinador con N=0", color='r')
    axs[1].plot(reference_index_ugv_1['time']/1e9, reference_index_uav_1['data'] - reference_index_ugv_1['data'], label="Error para coordinador con N=5", color='g')
    axs[1].plot(reference_index_ugv_2['time']/1e9, reference_index_uav_2['data'] - reference_index_ugv_2['data'], label="Error para coordinador con N=15", color='c')
    axs[1].plot(reference_index_ugv_3['time']/1e9, reference_index_uav_3['data'] - reference_index_ugv_3['data'], label="Error para coordinador con N=45", color='k')
    axs[1].axhline(y=0.0, color='k', linestyle='--')
    axs[1].set_title("Error de índices UAV-UGV")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("error de índice")
    axs[1].grid(True)
    axs[1].legend()

    # Primer conjunto de datos (curva roja en axs[1])
    t_error1 = reference_index_ugv['time'] 
    error1 = reference_index_uav['data'] - reference_index_ugv['data']

    # Segundo conjunto de datos (curva amarilla en axs[1])
    t_error2 = reference_index_ugv_1['time'] / 1e9
    error2 = reference_index_uav_1['data'] - reference_index_ugv_1['data']

    # Tercer conjunto de datos (curva verde en axs[1])
    t_error3 = reference_index_ugv_2['time'] / 1e9
    error3 = reference_index_uav_2['data'] - reference_index_ugv_2['data']  

    # Cuarto conjunto de datos (curva azul en axs[1])
    t_error4 = reference_index_ugv_3['time'] / 1e9
    error4 = reference_index_uav_3['data'] - reference_index_ugv_3['data']

    # Área total bajo las curvas (valor absoluto del error)
    area_total_error1 = np.trapz(np.abs(error1), t_error1)
    area_total_error2 = np.trapz(np.abs(error2), t_error2)
    area_total_error3 = np.trapz(np.abs(error3), t_error3)
    area_total_error4 = np.trapz(np.abs(error4), t_error4)

    print(f"Área total bajo la curva de error (conjunto 1, rojo): {area_total_error1:.4f}")
    print(f"Área total bajo la curva de error (conjunto 2, verde): {area_total_error2:.4f}")
    print(f"Área total bajo la curva de error (conjunto 3, azul): {area_total_error3:.4f}")
    print(f"Área total bajo la curva de error (conjunto 4, naranja): {area_total_error4:.4f}")
    plt.tight_layout()

def plot_speed_comparison(uav_speed_xy, uav_speed_z, ugv_speed, uav_speed_xy_1, uav_speed_z_1, ugv_speed_1):
    uav_speed_xy['time'] = uav_speed_xy['time'] / 1e9  # Convertir tiempo a segundos
    uav_speed_z['time'] = uav_speed_z['time'] / 1e9 
    ugv_speed['time'] = ugv_speed['time'] / 1e9  
    uav_speed_xy_1['time'] = uav_speed_xy_1['time'] / 1e9  
    uav_speed_z_1['time'] = uav_speed_z_1['time'] / 1e9 
    ugv_speed_1['time'] = ugv_speed_1['time'] / 1e9

    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # filas, columna
    fig.suptitle("Comparación de velocidades enviadas por el coordinador", fontsize=16)

        # Subplot 1: Velocidad del UGV
    axs[0].plot(ugv_speed['time'], ugv_speed['data'], label="Velocidad discreta para UGV", color='r')
    axs[0].plot(ugv_speed_1['time'], ugv_speed_1['data'], label="Velocidad continua para UGV", color='g')
    axs[0].set_title("Velocidad del UGV a lo largo del tiempo")
    axs[0].set_xlabel("t [s]")
    axs[0].set_ylabel("v [m/s]")
    axs[0].grid(True)
    axs[0].legend()

        # Subplot 2: Velocidad del UAV en el eje X e Y
    axs[1].plot(uav_speed_xy['time'], uav_speed_xy['data'], label="Velocidad discreta para UAV en XY", color='y')
    axs[1].plot(uav_speed_xy_1['time'], uav_speed_xy_1['data'], label="Velocidad continua para UAV en XY", color='b')
    axs[1].set_title("Velocidad del UAV en XY a lo largo del tiempo")
    axs[1].set_xlabel("t [s]")
    axs[1].set_ylabel("v [m/s]")
    axs[1].grid(True)
    axs[1].legend()

        # Subplot 3: Velocidad del UAV en el eje Z
    axs[2].plot(uav_speed_z['time'], uav_speed_z['data'], label="Velocidad para UAV en Z", color='b')
    axs[2].plot(uav_speed_z_1['time'], uav_speed_z_1['data'], label="Velocidad para UAV en Z", color='c')
    axs[2].set_title("Velocidad del UAV en Z a lo largo del tiempo")
    axs[2].set_xlabel("t [s]")
    axs[2].set_ylabel("v [m/s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()

# --- MAIN ---
def main():

    # === UAV ===
    waypoints_uav = np.array(load_waypoints_UAV())
    uav_pos = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/sjtu_drone_gt_pose.csv")
    reference_index_uav = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_waypoint_index.csv")

    # === UGV ===
    waypoints_ugv = np.array(load_waypoints_UGV())
    ugv_pos = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/rs_robot_ugv_gt_pose.csv")
    reference_index_ugv = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/ugv_waypoint_index.csv")

    # === CABLE ===
    waypoints_length = np.array(load_waypoints_length()) 
    cable_length = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/kp0.5/cable_length.csv")
    cable_length_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/kp1/cable_length.csv")
    cable_length_2 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/kp1.5/cable_length.csv")

    uav_speed_xy = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_reference_speed_xy.csv")
    uav_speed_z = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_reference_speed_z.csv")
    ugv_speed = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/ugv_reference_speed.csv")
    
    winch_velocity = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/forward_velocity_controller_commands.csv")

    # == LOOKAHEAD ADAPTATIVO == #
    #lh_uav = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/uav_distance.csv")
    #lh_ugv = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/ugv_distance.csv")

    reference_index_ugv_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=5/ugv_waypoint_index.csv") 
    reference_index_uav_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=5/uav_waypoint_index.csv")

    reference_index_ugv_2 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=0/ugv_waypoint_index.csv") 
    reference_index_uav_2 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=0/uav_waypoint_index.csv")

    reference_index_ugv_3 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=45/ugv_waypoint_index.csv") 
    reference_index_uav_3 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/N=45/uav_waypoint_index.csv")

    uav_speed_xy_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/coord_wpsync_lhadaptative/uav_reference_speed_xy.csv")
    uav_speed_z_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/coord_wpsync_lhadaptative/uav_reference_speed_z.csv")
    ugv_speed_1 = load_csv("/home/upo/marsupial/src/ros2_marsupial_coordinator/csv/cmp/coord_wpsync_lhadaptative/ugv_reference_speed.csv")

    # Gráficas

    # === UAV PLOTS ===
    plot_uav_trajectory_2D(uav_pos, waypoints_uav)
    plot_uav_trajectory_3D(uav_pos, waypoints_uav)

    plot_uav_error(uav_pos, waypoints_uav, reference_index_uav)
   # plot_uav_error_adaptative(uav_pos, waypoints_uav, reference_index)

    # === UGV PLOTS ===
    plot_ugv_trajectory_2D(ugv_pos, waypoints_ugv)
    plot_ugv_trajectory_3D(ugv_pos, waypoints_ugv)
    plot_ugv_error(ugv_pos, waypoints_ugv, reference_index_ugv)
    #plot_ugv_error_adaptative(ugv_pos, waypoints_ugv, reference_index_ugv, lh_ugv) # LH FIJO

    # === CABLE PLOTS ===
    plot_tether_trajectory(cable_length, winch_velocity, cable_length_1, cable_length_2)

    # === COORDINACION ===
    #plot_coordinator_performance(reference_index_ugv, reference_index_uav)
    #plot_coordinator_comparison(reference_index_ugv, reference_index_uav, reference_index_ugv_1, reference_index_uav_1, reference_index_uav_2, reference_index_ugv_2, reference_index_ugv_3, reference_index_uav_3)
    #plot_speed_comparison(uav_speed_xy, uav_speed_z, ugv_speed, uav_speed_xy_1, uav_speed_z_1, ugv_speed_1) 

    # == VELOCIDADES === 
    plot_speeds(ugv_speed, uav_speed_xy, uav_speed_z)

    plt.show()

if __name__ == "__main__":
    main()
