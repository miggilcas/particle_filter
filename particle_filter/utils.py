import numpy as np
from geometry_msgs.msg import Quaternion
import yaml
import cv2
import os
import time

def euler_from_quaternion(q):
    # Convertir quaternion (ROS) a ángulos de Euler
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(2*(w*y - z*x))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return [roll, pitch, yaw]

def quaternion_from_euler(roll, pitch, yaw):
    # Convertir Euler a quaternion (ROS)
    q = Quaternion()
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

def ray_cast(particle, map_data, map_resolution=0.05, map_origin=(0.0, 0.0), max_range=3.5, num_rays=30):
    """
    Simula lecturas láser desde una posición dada en un mapa de ocupación binaria.

    Args:
        particle: (x, y, theta) posición del robot.
        map_data: mapa de ocupación (2D numpy array, 0 libre, 1 ocupado).
        map_resolution: tamaño de cada celda en metros.
        map_origin: coordenadas del origen del mapa (en metros).
        max_range: rango máximo del láser.
        num_rays: número de rayos a simular (normalmente 360).

    Returns:
        distancias: array de tamaño (num_rays,) con las distancias simuladas.
    """
    x, y, theta = particle
    angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
    distances = np.full(num_rays, max_range)

    height, width = map_data.shape

    for i, angle in enumerate(angles):
        ray_angle = theta + angle
        for r in np.linspace(0, max_range, int(max_range / map_resolution)):
            # Coordenadas en el mundo
            x_r = x + r * np.cos(ray_angle)
            y_r = y + r * np.sin(ray_angle)

            # Índices en el mapa
            ix = int((x_r - map_origin[0]) / map_resolution)
            iy = int((y_r - map_origin[1]) / map_resolution)

            #if i == 0 and r == 0.0:  # Solo en el primer rayo, primer paso
            #   print(f"[ray_cast] Partícula: ({x:.2f}, {y:.2f}, {theta:.2f}) → Mapa: ({ix}, {iy})")

            if 0 <= ix < width and 0 <= iy < height:
                if map_data[iy, ix] == 0:
                    distances[i] = r
                    # 🔍 DEBUG: muestra la primera colisión de algunos rayos
                    #if i % (num_rays // 8) == 0:  # muestra 8 rayos solamente
                    #    print(f"Rayo {i:3}: colisión a {r:.2f} m en mapa ({ix}, {iy})")

                    break
            else:
                # El rayo salió del mapa
                distances[i] = r
                break

    return distances

def compute_likelihood(scan_ranges, particles, map_data, map_resolution=0.05, map_origin=(0.0, 0.0)):
    weights = []
    t0 = time.time()

    for i, particle in enumerate(particles):
        t1 = time.time()

        expected = ray_cast(particle, map_data, map_resolution=map_resolution, map_origin=map_origin, num_rays=len(scan_ranges))
        scan_ranges = np.array(scan_ranges)
        expected = np.array(expected)
        t2 = time.time()

        # Reemplaza inf/nan por el valor máximo del láser
        scan_ranges = np.clip(scan_ranges, 0.12, 3.5)
        expected = np.clip(expected, 0.12, 3.5)

        error = np.sum((np.array(scan_ranges) - expected) ** 2)
        weight = np.exp(-error/10.0)
        weights.append(weight)  # Suaviza este parámetro si quieres
        # 🔍 DEBUG: imprime para cada partícula lo esencial
        # print(f"\n[Partícula {i}] Pos: x={particle[0]:.2f}, y={particle[1]:.2f}, θ={np.degrees(particle[2]):.1f}°")
        # print(f"[->] Error cuadrático total: {error:.2f}")
        # print(f"[->] Peso: {weight:.6f}")
        
        # Solo para la primera partícula, muestra los primeros 10 rayos esperados vs reales
        #if i == 0:
        #    print("Rayo  | Real     | Esperado")
        #    for j in range(0, len(scan_ranges), len(scan_ranges)//10):
        #        print(f"{j:4}  | {scan_ranges[j]:.2f} m  | {expected[j]:.2f} m")

        if i == 0:
            print(f"Tiempo ray_cast: {t2 - t1:.4f} s")

    print(f"Tiempo total likelihood: {time.time() - t0:.4f} s")

    return np.array(weights)

def compute_effective_sample_size(weights):
    return 1.0 / np.sum(np.square(weights))

def compute_distances_to_beacons(beacons,robot_pose):
    """
    Calcula la distancia desde la pose del robot a cada baliza conocida.

    Args:
        robot_pose: tupla (x, y) en coordenadas del mundo (puede obtenerse desde la odometría)

    Returns:
        Lista de distancias a cada baliza
    """
    x, y = robot_pose
    distances = []
    for bx, by in beacons:
        d = np.hypot(bx - x, by - y)
        distances.append(d)
    return distances

def beacon_weight_correction(beacons, particles, real_distances):
    """
    Ajusta pesos comparando la distancia desde las partículas a las balizas con las reales.

    Args:
        particles: np.array de forma (N, 3)
        real_distances: distancias reales a cada baliza (lista de floats)

    Returns:
        np.array con factores multiplicativos para los pesos
    """
    beacon_std = 0.3  # incertidumbre de medición en metros
    weights = np.ones(len(particles))

    for i, (px, py, _) in enumerate(particles):
        for j, (bx, by) in enumerate(beacons):
            dist_particle = np.hypot(bx - px, by - py)
            dist_real = real_distances[j]
            error = dist_particle - dist_real
            weights[i] *= np.exp(-0.5 * (error / beacon_std) ** 2)

    return weights
def inject_random_particles(particles, map_data, map_resolution, map_origin, percentage=0.05):
    N = len(particles)
    n_new = int(N * percentage)
    new_particles = initialize_particles_clustered(
        num_particles=n_new,
        map_data=map_data,
        map_resolution=map_resolution,
        map_origin=map_origin,
        clusters=1
    )
    particles[:n_new] = new_particles
    return particles
