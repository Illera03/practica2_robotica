from glob import glob

from setuptools import find_packages, setup

package_name = 'practica_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dalinar',
    maintainer_email='ikerarga3@gmail.com',
    description='Ingesta, calibración y procesamiento de imágenes de cámara USB con ROS 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capturador_imagenes = practica_2.capturador_imagenes:main',
            'corrector_distorsion = practica_2.corrector_distorsion:main',
        ],
    },
)
