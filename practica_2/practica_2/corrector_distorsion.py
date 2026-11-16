"""
Aplica la calibración de la cámara con OpenCV (punto 4 de la práctica).

El nodo lee la matriz intrínseca "K" y los coeficientes de distorsión "D" del topic
"/camera_info" (que "usb_cam" rellena con el YAML de "camera_calibration"), convierte cada
imagen de "/image_raw" con "cv_bridge", le quita la distorsión y la publica en
"/image_rect".

Opcionalmente corrige también las imágenes guardadas por "capturador_imagenes" y las escribe
en otra carpeta, para comparar el antes y el después.

Uso:
    ros2 run practica_2 corrector_distorsion
    ros2 run practica_2 corrector_distorsion --ros-args -p directorio_capturas:=capturas
"""

import glob
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

TOPIC_IMAGEN_POR_DEFECTO: str = '/image_raw'
TOPIC_INFO_POR_DEFECTO: str = '/camera_info'
TOPIC_CORREGIDA_POR_DEFECTO: str = '/image_rect'
# Cadena vacía = no corregir ninguna carpeta
DIRECTORIO_CAPTURAS_POR_DEFECTO: str = ''
SUFIJO_DIRECTORIO_CORREGIDO: str = '_corregidas'
PATRON_IMAGENES: str = '*.png'

PROFUNDIDAD_COLA: int = 10
PERIODO_AVISO_S: float = 5.0
# 0 recorta los bordes negros que deja la corrección; 1 conserva todos los píxeles originales
ALFA_NUEVA_MATRIZ: float = 0.0


class CorrectorDistorsion(Node):
    """Nodo que publica la imagen de la cámara sin distorsión a partir de su calibración."""

    def __init__(self) -> None:
        """Lee los parámetros, crea el publicador y se suscribe a la imagen y a su calibración."""
        super().__init__('corrector_distorsion')

        topic_imagen: str = self.declare_parameter(
            'topic_imagen', TOPIC_IMAGEN_POR_DEFECTO).value
        topic_info: str = self.declare_parameter(
            'topic_info', TOPIC_INFO_POR_DEFECTO).value
        topic_corregida: str = self.declare_parameter(
            'topic_corregida', TOPIC_CORREGIDA_POR_DEFECTO).value
        self._directorio_capturas: str = self.declare_parameter(
            'directorio_capturas', DIRECTORIO_CAPTURAS_POR_DEFECTO).value

        self._puente: CvBridge = CvBridge()
        # Los mapas de corrección se calculan una vez por resolución y se reutilizan con remap,
        # mucho más barato que llamar a cv2.undistort en cada fotograma.
        self._mapa_x: np.ndarray | None = None
        self._mapa_y: np.ndarray | None = None
        self._tamano_mapas: tuple[int, int] | None = None
        self._k: np.ndarray | None = None
        self._d: np.ndarray | None = None

        self._publicador = self.create_publisher(Image, topic_corregida, PROFUNDIDAD_COLA)
        self.create_subscription(CameraInfo, topic_info, self._callback_info, PROFUNDIDAD_COLA)
        self.create_subscription(Image, topic_imagen, self._callback_imagen, PROFUNDIDAD_COLA)
        self.get_logger().info(
            f'Corrigiendo {topic_imagen} con {topic_info}; publicando en {topic_corregida}')

    def _callback_info(self, mensaje: CameraInfo) -> None:
        """Guarda K y D de la calibración y regenera los mapas si han cambiado."""
        k = np.array(mensaje.k, dtype=np.float64).reshape(3, 3)
        d = np.array(mensaje.d, dtype=np.float64)
        if not k.any():
            # usb_cam publica K a cero cuando no encuentra el YAML de calibración
            self.get_logger().warn(
                'La cámara no está calibrada (K = 0): falta el YAML de camera_calibration',
                throttle_duration_sec=PERIODO_AVISO_S)
            return
        if self._k is not None and np.array_equal(k, self._k) and np.array_equal(d, self._d):
            return

        primera_calibracion = self._k is None
        self._k, self._d = k, d
        self._tamano_mapas = None
        self.get_logger().info(f'Calibración recibida:\nK =\n{k}\nD = {d}')
        if primera_calibracion and self._directorio_capturas:
            self._corregir_directorio(self._directorio_capturas)

    def _callback_imagen(self, mensaje: Image) -> None:
        """Convierte la imagen con cv_bridge, le quita la distorsión y la publica."""
        if self._k is None:
            return
        imagen = self._puente.imgmsg_to_cv2(mensaje, desired_encoding='bgr8')
        corregida = self._corregir(imagen)
        salida = self._puente.cv2_to_imgmsg(corregida, encoding='bgr8')
        salida.header = mensaje.header
        self._publicador.publish(salida)

    def _corregir(self, imagen: np.ndarray) -> np.ndarray:
        """Quita la distorsión a una imagen BGR usando los mapas de su resolución."""
        alto, ancho = imagen.shape[:2]
        if self._tamano_mapas != (ancho, alto):
            self._calcular_mapas(ancho, alto)
        return cv2.remap(imagen, self._mapa_x, self._mapa_y, cv2.INTER_LINEAR)

    def _calcular_mapas(self, ancho: int, alto: int) -> None:
        """Calcula los mapas de corrección para una resolución con initUndistortRectifyMap."""
        nueva_k, _ = cv2.getOptimalNewCameraMatrix(
            self._k, self._d, (ancho, alto), ALFA_NUEVA_MATRIZ)
        self._mapa_x, self._mapa_y = cv2.initUndistortRectifyMap(
            self._k, self._d, None, nueva_k, (ancho, alto), cv2.CV_32FC1)
        self._tamano_mapas = (ancho, alto)

    def _corregir_directorio(self, directorio: str) -> None:
        """Corrige las imágenes PNG de una carpeta y las guarda en "<carpeta>_corregidas"."""
        origen = os.path.abspath(os.path.expanduser(directorio))
        destino = origen.rstrip(os.sep) + SUFIJO_DIRECTORIO_CORREGIDO
        rutas = sorted(glob.glob(os.path.join(origen, PATRON_IMAGENES)))
        if not rutas:
            self.get_logger().warn(f'No hay imágenes {PATRON_IMAGENES} en {origen}')
            return
        os.makedirs(destino, exist_ok=True)
        for ruta in rutas:
            imagen = cv2.imread(ruta)
            if imagen is None:
                self.get_logger().error(f'No se ha podido leer {ruta}')
                continue
            ruta_salida = os.path.join(destino, os.path.basename(ruta))
            if not cv2.imwrite(ruta_salida, self._corregir(imagen)):
                self.get_logger().error(f'No se ha podido escribir {ruta_salida}')
        self.get_logger().info(f'{len(rutas)} imágenes corregidas en {destino}')


def main(args: list[str] | None = None) -> None:
    """Ejecuta el corrector hasta Ctrl+C."""
    rclpy.init(args=args)
    corrector = CorrectorDistorsion()
    try:
        rclpy.spin(corrector)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        corrector.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
