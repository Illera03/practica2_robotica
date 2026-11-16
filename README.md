# practica2_robotica

Práctica 2 de Robótica: ingesta, calibración y procesamiento de imágenes de una cámara USB con
ROS 2 Jazzy. Todo el código está en el paquete `practica_2`.

## Preparar el entorno

1. La webcam tiene que estar dentro del contenedor. En el repositorio del laboratorio
   (`ros2_mucsi_26_27-main`) se añade un `docker-compose.override.yml` junto a
   `docker-compose.yml`:

   ```yaml
   services:
     desktop:
       devices:
         - /dev/video0:/dev/video0
         - /dev/video2:/dev/video2
   ```

   y se vuelve a ejecutar `./lab.sh iniciar`. Con `./lab.sh iniciar --lab` no hace falta, porque
   ese perfil ya monta todo `/dev`.

2. Dentro del contenedor hay que instalar el calibrador (se pierde si se recrea el contenedor):

   ```bash
   sudo apt update && sudo apt install -y ros-jazzy-camera-calibration
   ```

3. Compilar:

   ```bash
   cd ~/ros_workspace && colcon build --symlink-install --packages-select practica_2
   source install/setup.bash
   ```

## 1. Publicador de imágenes

```bash
ros2 launch practica_2 camara.launch.py
```

Lanza `usb_cam` (publica `/image_raw` y `/camera_info`) y `rqt_image_view`. Argumentos:
`video_device` (por defecto `/dev/video2`, la cámara USB; `/dev/video0` es la webcam integrada), `camera_info_url` y `mostrar_visor`.

## 2. Ingesta de 10 imágenes distintas

Con la cámara lanzada, en otra terminal:

```bash
ros2 run practica_2 capturador_imagenes
```

El nodo convierte cada mensaje con `cv_bridge` y solo guarda un fotograma si:

- es nítido (varianza del Laplaciano ≥ `nitidez_minima`), para descartar fotogramas movidos;
- se diferencia de **todas** las imágenes ya guardadas en más de `diferencia_minima` niveles de
  gris de media. Se comparan versiones reducidas y suavizadas para que el ruido del sensor no
  cuente como diferencia;
- opcionalmente, se ven al menos `marcadores_minimos` marcadores de la hoja ArUco de Visión
  (`DICT_4X4_50`, IDs 0-3). Por defecto no se exige.

Las imágenes se guardan como `capturas/imagen_01.png` … `imagen_10.png` (en la carpeta desde la
que se lanza) y el nodo termina solo. Parámetros:

| Parámetro | Por defecto | Descripción |
|---|---|---|
| `topic_imagen` | `/image_raw` | Topic de entrada |
| `numero_imagenes` | `10` | Imágenes a guardar |
| `directorio_salida` | `capturas` | Carpeta de salida |
| `diferencia_minima` | `10.0` | Diferencia media mínima con las ya guardadas (0-255) |
| `nitidez_minima` | `6.0` | Varianza del Laplaciano mínima |
| `marcadores_minimos` | `0` | Marcadores ArUco que deben verse (0 = no se exige) |

Ejemplo exigiendo que se vea la hoja de marcadores:

```bash
ros2 run practica_2 capturador_imagenes --ros-args -p marcadores_minimos:=3
```

## 3. Calibración de la cámara

*Pendiente.* El YAML resultante va en `practica_2/config/calibracion_camara.yaml`, que es el que
carga `camara.launch.py` por defecto.

## 4. Aplicación de la calibración

*Pendiente.*
