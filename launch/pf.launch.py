from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta al archivo YAML con parámetros
    pf_params_path = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'pf_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='particle_filter',
            executable='particle_filter',
            name='particle_filter',
            output='screen',
            parameters=[pf_params_path]
        )
    ])



