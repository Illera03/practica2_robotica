"""
Publicador de imágenes de la cámara USB (punto 1 de la práctica).

Lanza el nodo de ``usb_cam``, que publica ``/image_raw`` y ``/camera_info``, y opcionalmente
``rqt_image_view`` para ver el vídeo. Si existe el YAML de calibración en ``config/``, usb_cam
lo carga y lo publica en ``/camera_info``; si no existe, solo avisa y publica sin calibración.

Uso:
    ros2 launch practica_2 camara.launch.py
    ros2 launch practica_2 camara.launch.py video_device:=/dev/video2 mostrar_visor:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# En los portátiles la webcam integrada ocupa video0 y video1; la USB es la siguiente
DISPOSITIVO_CAMARA_USB: str = '/dev/video2'
ANCHO_IMAGEN: int = 640
ALTO_IMAGEN: int = 480
FORMATO_PIXEL: str = 'yuyv'
FRAME_CAMARA: str = 'usb_cam'
NOMBRE_ARCHIVO_CALIBRACION: str = 'calibracion_camara.yaml'
# usb_cam fuerza brightness=50 por defecto, pensado para cámaras con rango 0-100; en webcams con
# rango 0-255 la imagen sale casi negra. Con -1 usb_cam no toca el brillo de la cámara.
NO_MODIFICAR_CONTROL: int = -1


def generate_launch_description() -> LaunchDescription:
    """Construye la descripción del launch con la cámara y el visor."""
    video_device = LaunchConfiguration('video_device')
    camera_info_url = LaunchConfiguration('camera_info_url')
    mostrar_visor = LaunchConfiguration('mostrar_visor')

    # usb_cam espera una URL, por eso se antepone file:// a la ruta del paquete instalado
    ruta_calibracion = PathJoinSubstitution(
        [FindPackageShare('practica_2'), 'config', NOMBRE_ARCHIVO_CALIBRACION])

    camara = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'image_width': ANCHO_IMAGEN,
            'image_height': ALTO_IMAGEN,
            'pixel_format': FORMATO_PIXEL,
            'camera_frame_id': FRAME_CAMARA,
            'camera_name': FRAME_CAMARA,
            'camera_info_url': camera_info_url,
            'io_method': 'mmap',
            'brightness': NO_MODIFICAR_CONTROL,
        }],
    )

    visor = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='visor_imagen',
        output='screen',
        arguments=['/image_raw'],
        condition=IfCondition(mostrar_visor),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value=DISPOSITIVO_CAMARA_USB,
            description='Dispositivo V4L2 de la cámara (/dev/video0 es la webcam integrada)'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=['file://', ruta_calibracion],
            description='URL del YAML de calibración generado con camera_calibration'),
        DeclareLaunchArgument(
            'mostrar_visor', default_value='true',
            description='Abrir rqt_image_view con la imagen de la cámara'),
        camara,
        visor,
    ])
