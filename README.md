# particle_filter
Este repositorio contiene el código base para la Práctica 4 de la asignatura de Ampliación de Robótica, cuyo objetivo es implementar un Filtro de Partículas en un entorno simulado con ROS 2.
# Particle Filter ROS 2

Este repositorio contiene la implementación de un filtro de partículas (Particle Filter) en ROS 2 Humble para estimar la posición de un robot TurtleBot3 dentro de un mapa conocido, usando datos de odometría y láser.

## Requisitos

- ROS 2 Humble
- TurtleBot3 (paquetes instalados)
- Gazebo
- RViz2
- Python 3.10+

## Estructura del paquete
particle_filter/
├── config/
│ ├── pf_params.yaml # Parámetros del filtro
│ └── mapa.yaml # Mapa 2D del entorno
├── launch/
│ └── pf.launch.py # Lanzador del nodo
├── particle_filter/ # Código fuente Python
│ ├── init.py
│ ├── particle_filter.py # Nodo principal
│ └── utils.py # Funciones auxiliares

## Instalación

1. Clona el repositorio en tu espacio de trabajo ROS 2:

```bash
mkdir -p ~/AdR/p4_ws/src
cd ~/AdR/p4_ws/src
git clone https://github.com/miggilcas/particle_filter
```

2. Compilar el paquete
```bash
cd ~/AdR/p4_ws
colcon build --packages-select particle_filter
source install/setup.bash
```

## Ejecución
### 1. Simulador Gazebo
Lanza el mundo de simulación de TurtleBot3:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


```
### 2. Transformación estática
Publica la transformación estática entre odom y map:

```bash
ros2 run tf2_ros static_transform_publisher -2 -0.5 0 0 0 0 odom map

```
### 3. RViz2
Abre el visualizador:
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py

```
Configura RViz2:

- Fixed Frame: map
- Añadir visualización de tipo PoseArray
- Topic: /particle_cloud

### 4. Filtro de partículas
Lanza el nodo principal que hará correr el filtro de partículas:
```bash
ros2 launch particle_filter pf.launch.py

```
Esto ejecutará el nodo que lee datos de odometría y láser, procesa la información y publica una nube de partículas estimando la posición del robot.

## Parámetros
Ubicados en config/pf_params.yaml:
```yaml
particle_filter_node:
  ros__parameters:
    particle_count: 20

    motion_noise:
      x: 0.1
      y: 0.1
      theta: 0.05

    map_file: "mapa.yaml"

    topics:
      odom: "/odom"
      scan: "/scan"
      particles: "/particle_cloud"
```
## Tmuxinator
Esta herramienta nos servirá para automatizar el lanzado de todos los terminales y nos será mucho más cómodo y útil para proyectos de mayor complejidad.

```bash
sudo apt install tmux tmuxinator
cd ~/AdR/p4_student_ws/particle_filter
tmuxinator start -p pf_auto.yml

``` 
