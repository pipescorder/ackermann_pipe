from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_pipe'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'imgs'), glob('ackermann_pipe/imgs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Escobar',
    maintainer_email='gabriel.escobar1901@alumnos.ubiobio.cl',
    description='Simulación y visualización de robot Ackermann con LIDAR en ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
