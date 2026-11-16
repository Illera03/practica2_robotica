"""
Suscriptor que ingiere y almacena 10 imágenes distintas de la cámara (punto 2 de la práctica).

El nodo escucha el topic de ``usb_cam``, convierte cada mensaje con ``cv_bridge`` y guarda el
fotograma solo si es nítido y claramente distinto de todos los que ya se han guardado. Al
llegar al número pedido, termina.

Uso:
    ros2 run practica_2 capturador_imagenes
    ros2 run practica_2 capturador_imagenes --ros-args -p marcadores_minimos:=3
"""

import os

import cv2
from cv_bridge import CvBridge
import numpy as np
from practica_2.analisis_imagen import (
    a_escala_grises,
    calcular_diferencia,
    calcular_huella,
    calcular_nitidez,
    DetectorAruco,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC_IMAGEN_POR_DEFECTO: str = '/image_raw'
NUMERO_IMAGENES_POR_DEFECTO: int = 10
DIRECTORIO_SALIDA_POR_DEFECTO: str = 'capturas'
# Niveles de gris de diferencia media entre huellas. El ruido de una cámara quieta da menos de
# 2; mover la cámara o el objeto unos centímetros supera 10 con holgura.
DIFERENCIA_MINIMA_POR_DEFECTO: float = 10.0
# Mismo umbral de nitidez que capturar.py / calibrar.py de Visión
NITIDEZ_MINIMA_POR_DEFECTO: float = 6.0
# 0 = no exigir que se vea la hoja ArUco
MARCADORES_MINIMOS_POR_DEFECTO: int = 0

PROFUNDIDAD_COLA: int = 10
PERIODO_AVISO_DESCARTE_S: float = 2.0
PERIODO_ESPERA_SPIN_S: float = 0.1
EXTENSION_IMAGEN: str = 'png'


class CapturadorImagenes(Node):
    """Nodo que guarda una serie de imágenes diferentes entre sí a partir de un topic."""

    def __init__(self) -> None:
        """Lee los parámetros, prepara la carpeta de salida y se suscribe al topic."""
        super().__init__('capturador_imagenes')

        topic_imagen: str = self.declare_parameter(
            'topic_imagen', TOPIC_IMAGEN_POR_DEFECTO).value
        self._numero_imagenes: int = self.declare_parameter(
            'numero_imagenes', NUMERO_IMAGENES_POR_DEFECTO).value
        self._diferencia_minima: float = self.declare_parameter(
            'diferencia_minima', DIFERENCIA_MINIMA_POR_DEFECTO).value
        self._nitidez_minima: float = self.declare_parameter(
            'nitidez_minima', NITIDEZ_MINIMA_POR_DEFECTO).value
        self._marcadores_minimos: int = self.declare_parameter(
            'marcadores_minimos', MARCADORES_MINIMOS_POR_DEFECTO).value
        directorio_salida: str = self.declare_parameter(
            'directorio_salida', DIRECTORIO_SALIDA_POR_DEFECTO).value

        self._directorio_salida: str = os.path.abspath(os.path.expanduser(directorio_salida))
        os.makedirs(self._directorio_salida, exist_ok=True)

        self._puente: CvBridge = CvBridge()
        self._detector_aruco: DetectorAruco = DetectorAruco()
        # Las imágenes se guardan en disco y también en memoria, junto con su huella, para
        # compararlas con los fotogramas siguientes sin volver a leerlas.
        self.imagenes: list[np.ndarray] = []
        self._huellas: list[np.ndarray] = []

        self.create_subscription(Image, topic_imagen, self._callback_imagen, PROFUNDIDAD_COLA)
        self.get_logger().info(
            f'Esperando imágenes en {topic_imagen}; se guardarán {self._numero_imagenes} '
            f'en {self._directorio_salida}')

    @property
    def completado(self) -> bool:
        """Indica si ya se han almacenado todas las imágenes pedidas."""
        return len(self.imagenes) >= self._numero_imagenes

    def _callback_imagen(self, mensaje: Image) -> None:
        """Convierte el mensaje a OpenCV y lo almacena si supera los filtros."""
        if self.completado:
            return
        imagen = self._puente.imgmsg_to_cv2(mensaje, desired_encoding='bgr8')
        imagen_gris = a_escala_grises(imagen)
        huella = calcular_huella(imagen_gris)

        motivo_descarte = self._motivo_descarte(imagen_gris, huella)
        if motivo_descarte is not None:
            # Llegan ~30 fotogramas por segundo: se avisa con poca frecuencia para no saturar
            self.get_logger().info(
                f'Fotograma descartado: {motivo_descarte}',
                throttle_duration_sec=PERIODO_AVISO_DESCARTE_S)
            return
        self._almacenar(imagen, huella)

    def _motivo_descarte(self, imagen_gris: np.ndarray, huella: np.ndarray) -> str | None:
        """
        Comprueba si un fotograma es apto para guardarse.

        :return: ``None`` si se puede guardar o una explicación de por qué no.
        """
        nitidez = calcular_nitidez(imagen_gris)
        if nitidez < self._nitidez_minima:
            return f'imagen borrosa (nitidez {nitidez:.1f} < {self._nitidez_minima})'

        # Se compara con todas las guardadas y no solo con la última, para que no se repita una
        # vista anterior si la cámara vuelve a la misma posición.
        if self._huellas:
            diferencia = min(calcular_diferencia(huella, h) for h in self._huellas)
            if diferencia < self._diferencia_minima:
                return (f'demasiado parecida a una ya guardada '
                        f'(diferencia {diferencia:.1f} < {self._diferencia_minima})')

        # La detección ArUco es lo más costoso, por eso se deja para el final
        if self._marcadores_minimos > 0:
            marcadores = self._detector_aruco.contar_marcadores(imagen_gris)
            if marcadores < self._marcadores_minimos:
                return f'solo se ven {marcadores}/{self._marcadores_minimos} marcadores ArUco'
        return None

    def _almacenar(self, imagen: np.ndarray, huella: np.ndarray) -> None:
        """Guarda la imagen en memoria y como PNG sin pérdida en la carpeta de salida."""
        indice = len(self.imagenes) + 1
        ruta = os.path.join(self._directorio_salida, f'imagen_{indice:02d}.{EXTENSION_IMAGEN}')
        if not cv2.imwrite(ruta, imagen):
            self.get_logger().error(f'No se ha podido escribir {ruta}')
            return
        self.imagenes.append(imagen)
        self._huellas.append(huella)
        self.get_logger().info(f'[{indice}/{self._numero_imagenes}] Guardada {ruta}')


def main(args: list[str] | None = None) -> None:
    """Ejecuta el capturador hasta reunir todas las imágenes o hasta Ctrl+C."""
    rclpy.init(args=args)
    capturador = CapturadorImagenes()
    try:
        # spin_once en lugar de spin para poder salir solos al completar la serie
        while rclpy.ok() and not capturador.completado:
            rclpy.spin_once(capturador, timeout_sec=PERIODO_ESPERA_SPIN_S)
        capturador.get_logger().info('Captura completada')
    except KeyboardInterrupt:
        pass
    finally:
        capturador.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
