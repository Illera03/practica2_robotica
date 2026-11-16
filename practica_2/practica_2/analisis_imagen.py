"""
Medidas sobre imágenes de OpenCV para decidir qué fotogramas merece la pena guardar.

No depende de ROS, así que se puede probar con imágenes sueltas. Los criterios de nitidez
(varianza del Laplaciano) y la hoja de marcadores ArUco (``DICT_4X4_50``, IDs 0-3) son los
mismos que usan las herramientas de calibración de la asignatura de Visión.
"""

from collections.abc import Callable, Sequence

import cv2
import numpy as np

# Resolución a la que se comparan las imágenes. Al reducirlas desaparece el detalle fino, que
# entre dos fotogramas de una cámara quieta es casi todo ruido del sensor.
TAMANO_COMPARACION: tuple[int, int] = (160, 120)
SIGMA_SUAVIZADO_COMPARACION: float = 1.0

DICCIONARIO_ARUCO_HOJA: str = 'DICT_4X4_50'
IDS_ARUCO_HOJA: tuple[int, ...] = (0, 1, 2, 3)


def a_escala_grises(imagen_bgr: np.ndarray) -> np.ndarray:
    """Convierte una imagen BGR de OpenCV a escala de grises."""
    return cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2GRAY)


def calcular_nitidez(imagen_gris: np.ndarray) -> float:
    """
    Mide la nitidez como la varianza del Laplaciano.

    El Laplaciano responde a los bordes. En una imagen movida o desenfocada los bordes se
    difuminan y su varianza cae, así que un valor bajo indica un fotograma poco útil.
    """
    return float(cv2.Laplacian(imagen_gris, cv2.CV_64F).var())


def calcular_huella(imagen_gris: np.ndarray) -> np.ndarray:
    """
    Reduce y suaviza una imagen para compararla con otras.

    La huella resultante conserva la composición de la escena (dónde está cada cosa) y descarta
    el ruido, de modo que dos fotogramas solo difieren si la escena o la cámara se han movido.
    """
    reducida = cv2.resize(imagen_gris, TAMANO_COMPARACION, interpolation=cv2.INTER_AREA)
    suavizada = cv2.GaussianBlur(reducida, (0, 0), SIGMA_SUAVIZADO_COMPARACION)
    return suavizada.astype(np.float32)


def calcular_diferencia(huella_a: np.ndarray, huella_b: np.ndarray) -> float:
    """Diferencia media absoluta entre dos huellas, en niveles de gris (0-255)."""
    return float(np.mean(np.abs(huella_a - huella_b)))


class DetectorAruco:
    """Cuenta los marcadores de la hoja de calibración ArUco que aparecen en una imagen."""

    def __init__(self, nombre_diccionario: str = DICCIONARIO_ARUCO_HOJA,
                 ids_validos: Sequence[int] = IDS_ARUCO_HOJA) -> None:
        """
        Prepara el detector para un diccionario predefinido de OpenCV.

        :param nombre_diccionario: nombre de la constante de ``cv2.aruco``, p. ej. ``DICT_4X4_50``.
        :param ids_validos: IDs que se cuentan. Se ignoran los marcadores ajenos a la hoja.
        """
        diccionario = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, nombre_diccionario))
        self._ids_validos: frozenset[int] = frozenset(ids_validos)
        self._detectar: Callable[[np.ndarray], tuple] = self._crear_funcion_deteccion(diccionario)

    @staticmethod
    def _crear_funcion_deteccion(diccionario: 'cv2.aruco.Dictionary'
                                 ) -> Callable[[np.ndarray], tuple]:
        """
        Devuelve la función de detección adecuada para la versión de OpenCV instalada.

        OpenCV 4.7 sustituyó ``cv2.aruco.detectMarkers`` por la clase ``ArucoDetector``. El
        contenedor del laboratorio trae OpenCV 4.6, pero los scripts de Visión usan la API
        nueva, así que se admiten las dos.
        """
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(diccionario, cv2.aruco.DetectorParameters())
            return detector.detectMarkers
        parametros = cv2.aruco.DetectorParameters_create()
        return lambda imagen_gris: cv2.aruco.detectMarkers(
            imagen_gris, diccionario, parameters=parametros)

    def contar_marcadores(self, imagen_gris: np.ndarray) -> int:
        """Número de marcadores distintos de la hoja detectados en la imagen."""
        _, ids, _ = self._detectar(imagen_gris)
        if ids is None:
            return 0
        return len(self._ids_validos.intersection(int(i) for i in ids.ravel()))
