#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from .utils import *  # Funciones auxiliares (más abajo)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory

class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter')  # <- Esto debe ir al principio

        # === Leer parámetros ===
        self.declare_parameter('particle_count', 100)
        self.declare_parameter('motion_noise.x', 0.1)
        self.declare_parameter('motion_noise.y', 0.1)
        self.declare_parameter('motion_noise.theta', 0.05)
        self.declare_parameter('map_file', 'mapa.yaml')
        self.declare_parameter('num_rays_used', 100)
        # self.declare_parameter('topics.odom', '/odom')
        # self.declare_parameter('topics.scan', '/scan')
        # self.declare_parameter('topics.particles', '/particle_cloud')

        # === Obtener parámetros ===
        self.particle_count = self.get_parameter('particle_count').get_parameter_value().integer_value
        self.motion_noise = {
            'x': self.get_parameter('motion_noise.x').get_parameter_value().double_value,
            'y': self.get_parameter('motion_noise.y').get_parameter_value().double_value,
            'theta': self.get_parameter('motion_noise.theta').get_parameter_value().double_value
        }
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.num_rays_used = self.get_parameter('num_rays_used').get_parameter_value().integer_value

        # Si decides usar topics personalizados, descomenta estas líneas
        # self.topic_odom = self.get_parameter('topics.odom').get_parameter_value().string_value
        # self.topic_scan = self.get_parameter('topics.scan').get_parameter_value().string_value
        # self.topic_particles = self.get_parameter('topics.particles').get_parameter_value().string_value

        self.topic_odom = '/odom'
        self.topic_scan = '/scan'
        self.topic_particles = '/particle_cloud'

        # === Mostrar parámetros por consola ===
        self.get_logger().info(f'Usando {self.particle_count} partículas')
        self.get_logger().info(f'Ruido de movimiento: {self.motion_noise}')
        self.get_logger().info(f'Mapa cargado desde: {self.map_file}')
        self.get_logger().info(f'Topics: odom={self.topic_odom}, scan={self.topic_scan}, cloud={self.topic_particles}')
        self.get_logger().info(f'Con {self.num_rays_used} rayos usados')


        # === Cargar mapa ===
        # Leer parámetro de mapa (puede ser relativo)

        map_param = self.get_parameter('map_file').value
        map_full_path = os.path.join(get_package_share_directory('particle_filter'), 'config', map_param)


        self.get_logger().info(f'Cargando archivo YAML de mapa desde: {map_full_path}')

        try:
            self.map_data, self.map_resolution, self.map_origin = self.load_map(map_full_path)
        except Exception as e:
            self.get_logger().error(f'Error cargando el mapa: {e}')
            rclpy.shutdown()
            return

        # === Inicializar partículas ===
        ## uniforme:
        self.particles = self.init_particles()
        #self.particles = self.initialize_particles_clustered(
        #    num_particles=self.num_particles,
        #    map_data=self.map_data,
        #    map_resolution=self.map_resolution,
        #    map_origin=self.map_origin,
        #    clusters=5 # a toquetiar
        #)

        # === Suscripciones y publicaciones ===
        self.create_subscription(Odometry, self.topic_odom, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.topic_scan, self.scan_callback, 10)
        self.particles_pub = self.create_publisher(PoseArray, self.topic_particles, 10)

        self.beacons = [(0.0, 0.0), (3.0, -1.2)]  # ajusta según tu mapa
        self.distances = [0.0, 0.0]

        print("Valores únicos del mapa:", np.unique(self.map_data))


        self.weights = np.ones(self.particle_count) / self.particle_count
        # Publicar partículas para RViz
        self.publish_particles()
        self.last_odom = None          # ← guarda la odometría previa




    def load_map(self, map_yaml_path):
        """
        Carga el mapa de ocupación a partir de un archivo YAML y una imagen PGM.

        Args:
        map_yaml_path (str): Ruta al archivo .yaml que describe el mapa.

        Returns:
        map_data (np.array): Array de ocupación del mapa (0 libre, 1 ocupado).
        resolution (float): Resolución del mapa en metros por celda.
        origin (tuple): Coordenadas del origen del mapa (x, y, theta).
        """
        self.get_logger().info(f"Cargando archivo YAML de mapa desde: {map_yaml_path}")

        # Cargar archivo YAML
        with open(map_yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)
        
        self.get_logger().info(f"Contenido del YAML: {map_info}")

        # Cargar la imagen PGM
        #image_path = map_info['image']
        #map_data = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        # Ruta absoluta de la imagen
        map_dir = os.path.dirname(map_yaml_path)
        image_rel_path = map_info['image']
        image_path = os.path.join(map_dir, image_rel_path)
        self.get_logger().info(f"Ruta de imagen PGM a cargar: {image_path}")

        # Cargar imagen
        map_data = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if map_data is None:
            raise FileNotFoundError(f"No se pudo cargar la imagen del mapa: {image_path}")


        self.get_logger().info(f"Mapa cargado con dimensiones: {map_data.shape}")

        # Convertir la imagen en mapa binario basado en los umbrales de ocupación
        occupied_thresh = map_info.get('occupied_thresh', 0.65)
        free_thresh = map_info.get('free_thresh', 0.25)

        self.get_logger().info(f"Umbrales: ocupado > {occupied_thresh}, libre < {free_thresh}")

        # El mapa tiene valores entre 0-255 (escala de grises), convertimos a 0 y 1
        map_data = np.where(map_data > occupied_thresh * 255, 1, 0)

        # Extraer la resolución y el origen
        resolution = map_info.get('resolution', 0.05)
        origin = map_info.get('origin', [-1.2, -2.36, 0])
        self.get_logger().info(f"Resolución del mapa: {resolution}, origen: {origin}")

        # (Opcional) Guardar el mapa binarizado para inspección
        cv2.imwrite('/home/student/AdR/mapa_binarizado_debug.png', map_data * 255)
        #cv2.imshow("Mapa cargado", map_data * 255)
        #cv2.waitKey(1000)  # Mostrar por 1 segundo

        return map_data, resolution, origin
    def init_particles(self):
        # Usar límites del mapa (ajusta según tu mapa real)
        map_width = self.map_data.shape[1] * self.map_resolution + self.map_origin[0]
        map_height = self.map_data.shape[0] * self.map_resolution + self.map_origin[1]
        
        particles = np.random.uniform(
            low=[self.map_origin[0],self.map_origin[1],-np.pi], 
            high=[map_width, map_height, np.pi],
            size=(self.get_parameter('particle_count').value, 3)
        )
        return particles
    def initialize_particles_clustered(self,num_particles, map_data, map_resolution, map_origin, clusters=4):
        """
        Inicializa partículas en varios clústeres separados dentro de zonas libres del mapa.
        """
        height, width = map_data.shape
        free_cells = np.argwhere(map_data == 1)  # zonas libres

        particles = []

        for _ in range(clusters):
            # Elegir una celda libre aleatoria
            idx = np.random.choice(len(free_cells))
            y_cell, x_cell = free_cells[idx]
            
            # Convertir a coordenadas del mundo
            x_world = x_cell * map_resolution + map_origin[0]
            y_world = y_cell * map_resolution + map_origin[1]

            # Añadir partículas con dispersión en torno al punto
            for _ in range(num_particles // clusters):
                px = x_world + np.random.normal(0, 0.2)
                py = y_world + np.random.normal(0, 0.2)
                pt = np.random.uniform(-np.pi, np.pi)
                particles.append([px, py, pt])

        return np.array(particles)
    def odom_callback(self, msg):
        # Predicción: mover partículas según odometría + ruido
        # pose absoluta de la odometría
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"[ODOM] Posición original (odom): x={x:.2f}, y={y:.2f}")
        # Pasamos la pose de odom a map
        x += (2)
        y += (0.5)
        self.get_logger().info(f"[ODOM] Posición estimada (map):   x={x:.2f}, y={y:.2f}")


        theta = euler_from_quaternion(msg.pose.pose.orientation)[2]

        if self.last_odom is None:          # primera llamada: solo guardo y salgo
            self.last_odom = (x, y, theta)
            return


         # Δ entre lecturas sucesivas
        delta_x   = x - self.last_odom[0]
        delta_y   = y - self.last_odom[1]
        delta_theta  = theta - self.last_odom[2]

        self.last_odom = (x, y, theta)      # actualizo para la próxima vez

        self.get_logger().info(f"Odometry recibida: Δx={delta_x:.2f}, Δy={delta_y:.2f}, Δθ={delta_theta:.2f}")

        noise = np.random.normal(0, [self.get_parameter('motion_noise.x').value, 
                                    self.get_parameter('motion_noise.y').value, 
                                    self.get_parameter('motion_noise.theta').value], 
                                    size=self.particles.shape)
        self.particles += np.array([delta_x, delta_y, delta_theta]) + noise

        # TODO Podemos calcular distancias a las balizas aquí:
        robot_pose = (x, y)
        self.distances = compute_distances_to_beacons(self.beacons,robot_pose)

        for i, d in enumerate(self.distances):
            self.get_logger().info(f"→ Distancia estimada a baliza {i}: {d:.2f} m (desde x={x:.2f}, y={y:.2f})")


    
    def scan_callback(self, msg):
         # Info sobre el LIDAR
        num_rays = len(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        range_max = msg.range_max
        range_min = msg.range_min

        self.get_logger().info(
            f"LIDAR: {num_rays} rayos | Ángulo: [{np.degrees(angle_min):.1f}°, {np.degrees(angle_max):.1f}°] "
            f"| Δθ={np.degrees(angle_increment):.2f}° | Rango: [{range_min:.2f}, {range_max:.2f}] m"
        )
        # Submuestreo uniforme
        # num_rays_used = 30
        selected_indices = np.linspace(0, num_rays - 1, self.num_rays_used, dtype=int)
        lidar_subset = np.array(msg.ranges)[selected_indices]

        # Corrección: actualizar pesos según LIDAR (comparar con mapa)
        ranges = np.array(msg.ranges)
        #self.weights = compute_likelihood(ranges, self.particles, self.map_data, map_resolution=self.map_resolution, map_origin=self.map_origin)  # Usar utils.py
        # Actualizar pesos usando solo las medidas seleccionadas
        self.weights = compute_likelihood(
            lidar_subset,
            self.particles,
            self.map_data,
            map_resolution=self.map_resolution,
            map_origin=self.map_origin,
        )

        ## Añadimos balizas con posiciones conocidas para evitar problemas por simetría
        # Pesos adicionales con balizas
        beacon_factors = beacon_weight_correction(self.beacons, self.particles, self.distances)
        self.weights *= beacon_factors   # combinación multiplicativa


        total_weight = np.sum(self.weights)

        self.get_logger().info(f"Suma de pesos: {total_weight:.4f}")

        if total_weight == 0 or np.isnan(total_weight):
            self.get_logger().warn("Todas las partículas tienen peso cero. Reasignando pesos uniformes.")
            self.weights = np.ones(len(self.particles)) / len(self.particles)
        else:
            self.weights /= total_weight
        
        # Remuestreo (resampling sistemático)
        self.dummy_resample()
        # self.systematic_resample()
        
        # Publicar partículas para RViz
        self.publish_particles()
    
    def dummy_resample(self):
        # Algoritmo sencillo (hay mejores como low-variance resampling)
        indices = np.random.choice(len(self.particles), size=len(self.particles), p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(len(self.particles)) / len(self.particles)
        unique = len(np.unique(indices))
        self.get_logger().info(f"Remuestreo completado. Partículas únicas: {unique}/{len(indices)}")

    
    def systematic_resample(self, threshold_ratio=0.5):
        """
        N = len(self.particles)
        positions = (np.arange(N) + np.random.uniform()) / N
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # evitar errores por redondeo
        indexes = np.zeros(N, dtype=int)

        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        self.particles = self.particles[indexes]
        self.weights = np.ones(N) / N
        unique = len(np.unique(indexes))
        self.get_logger().info(f"[Systematic] Remuestreo completado. Partículas únicas: {unique}/{N}")
        """

        N = len(self.particles)
        n_eff = compute_effective_sample_size(self.weights)
        threshold = threshold_ratio * N

        self.get_logger().info(f"[Systematic] N_eff: {n_eff:.2f} / {N} (umbral={threshold:.1f})")

        if n_eff < threshold:
            positions = (np.arange(N) + np.random.uniform()) / N
            cumulative_sum = np.cumsum(self.weights)
            cumulative_sum[-1] = 1.0  # evitar errores numéricos
            indexes = np.zeros(N, dtype=int)

            i, j = 0, 0
            while i < N:
                if positions[i] < cumulative_sum[j]:
                    indexes[i] = j
                    i += 1
                else:
                    j += 1

            self.particles = self.particles[indexes]
            self.weights = np.ones(N) / N
            unique = len(np.unique(indexes))
            self.get_logger().info(f"[Systematic] Remuestreo ejecutado. Partículas únicas: {unique}/{N}")
        else:
            self.get_logger().info(f"[Systematic] Sin remuestreo (suficiente diversidad).")
    
    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = 0.2
            pose.orientation = quaternion_from_euler(0, 0, p[2])  # Usar utils.py
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
