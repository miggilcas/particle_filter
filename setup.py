from setuptools import find_packages, setup

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/particle_filter/launch', ['launch/pf.launch.py']),
        ('share/particle_filter/config', ['config/pf_params.yaml']),
        ('share/particle_filter/config', ['config/mapa.yaml']),
        ('share/particle_filter/config', ['config/mapa.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Particle Filter Localization with LIDAR in ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter = particle_filter.particle_filter:main',

        ],
    },
)
