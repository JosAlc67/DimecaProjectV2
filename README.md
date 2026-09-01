# DIMECA — Celda Robótica de Recubrimiento (ABB IRB 2600 en ROS 2)

Este repositorio contiene la implementación y simulación en ROS 2 (Humble) y MoveIt 2 de una celda robótica diseñada para aplicaciones de recubrimiento (coating) sobre paneles curvos. El sistema está basado en el brazo industrial ABB IRB 2600, montado sobre un riel lineal para extender su espacio de trabajo.

El sistema es capaz de generar trayectorias de cobertura dinámicas adaptadas a la superficie de un panel en un entorno 3D, evadiendo obstáculos y planificando el movimiento de forma segura mediante trayectorias cartesianas suaves.

## Características Principales

*   **Modelo Cinemático Completo (URDF/Xacro):** Integración de un riel base (track) de 4.5 metros, la plataforma móvil, el robot ABB IRB 2600 y una herramienta de boquilla de pintura (TCP). Todo en un sistema de 7 grados de libertad (DOF).
*   **MoveIt 2 y Evasión de Colisiones Avanzada:** Entorno de planificación configurado con SRDF y objetos dinámicos inyectados. El sistema planifica dinámicamente rutas para asegurar que la estructura física del robot esquive los obstáculos en tiempo real.
*   **Planificación Dinámica 3D (Coverage Path Planning):** Generación automática de waypoints mediante raycasting usando la geometría del panel objetivo (STL). Mantiene un *standoff* (distancia focal) constante de 15 cm, estrictamente perpendicular a la superficie curva.
*   **Seguimiento Dinámico de Entorno:** El planificador consume la pose en tiempo real del lienzo publicada por el nodo de percepción, asegurando una aplicación de pintura en el lugar exacto sin depender de coordenadas rígidas.
*   **Ejecución Cartesiana Suave:** Ejecución interpolada linealmente en el espacio 3D para evitar reconfiguraciones drásticas de las articulaciones del brazo, asegurando un movimiento de pintura realista y fluido.
*   **Panel de Control GUI y Marcadores Interactivos:** Interfaz gráfica intuitiva desarrollada en Tkinter para un manejo simplificado del sistema (Inicio, Parada de Emergencia, Retorno a Casa). Integración con Interactive Markers en RViz para arrastrar y reposicionar obstáculos fácilmente durante la simulación.

## Requisitos del Sistema

*   **SO:** Ubuntu 22.04 LTS
*   **ROS:** ROS 2 Humble
*   **Dependencias Adicionales:**
    ```bash
    sudo apt update
    sudo apt install -y \
      ros-humble-desktop \
      ros-humble-moveit \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher-gui \
      python3-colcon-common-extensions python3-rosdep \
      python3-tk
    ```

## Instalación y Compilación

1. Clonar el repositorio dentro de un workspace de ROS 2 (`ros2_ws/src`):
   ```bash
   git clone https://github.com/JosAlc67/DimecaProjectV2.git
   ```
2. Instalar las dependencias de los paquetes usando `rosdep`:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Compilar el espacio de trabajo:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Uso y Ejecución

### 1. Inicializar la Celda de Simulación
Para levantar el entorno completo (RViz, MoveIt, controladores, escena CAD y panel de control), abre una terminal y ejecuta:
```bash
ros2 launch irb2600_coating_cell coating_cell_bringup.launch.py
```

### 2. Panel de Control (GUI)
El panel se abre automáticamente junto con la simulación. Si necesitas
iniciarlo de forma independiente, abre una segunda terminal con el workspace
compilado y ejecuta `ros2 run irb2600_coating_cell gui_control_node`.
Desde esta ventana podrás:
*   **Movimiento demo:** Realiza un desplazamiento corto, visible y validado por colisiones para comprobar el control y la animación en RViz.
*   **Start Route:** Iniciar el proceso de recubrimiento sobre el panel curvo, con la opción de definir la cantidad de pasadas (capas) necesarias mediante un selector.
*   **Stop (Emergency Stop):** Detiene el movimiento del robot y del riel instantáneamente en caso de emergencia, cancelando la ruta actual.
*   **Go Home:** Regresa el brazo robótico y la plataforma del riel a su posición inicial de reposo.

### 3. Modificación del Entorno (Obstáculos)
El sistema incluye marcadores interactivos integrados directamente en RViz. Podrás visualizar flechas de control (roja/verde/azul) sobre los obstáculos virtuales. 
Al arrastrarlos con el ratón hacia el área de trabajo del robot (frente al lienzo curvo), el entorno de colisión de MoveIt se actualizará en tiempo real. Al iniciar una ruta de pintura, el robot detectará estos obstáculos y re-planificará su trayectoria para esquivarlos de forma segura, interrumpiendo el flujo de pintura temporalmente.

## Arquitectura de Paquetes

*   **`irb2600_description`**: Contiene los archivos URDF/Xacro, mallas visuales y de colisión del brazo IRB 2600, el riel y la boquilla. Está configurado para usarse con `ros2_control` en un entorno de simulación (hardware mock).
*   **`irb2600_moveit_config`**: Paquete de configuración de MoveIt 2 que incluye la cinemática, límites articulares, controladores para los 7 DOF y los archivos `launch` de visualización base.
*   **`irb2600_coating_cell`**: Paquete central que contiene toda la lógica de control desarrollada en Python:
    *   `gui_control_node.py`: Panel de control GUI de usuario.
    *   `scene_setup_node.py`: Gestión y carga de obstáculos y geometría en la escena virtual.
    *   `perception_sim_node.py`: Simulador de percepción que publica la posición espacial de los obstáculos en la celda.
    *   `replanning_executor_node.py`: Núcleo matemático responsable de generar la trayectoria topográfica sobre la malla y exigir el movimiento cartesiano a MoveIt.
    *   `go_home_node.py`: Nodo de utilidad para enviar de manera segura el robot y su riel al origen espacial (cero absoluto).

## Créditos y Referencias

El URDF y las mallas visuales/colisión del robot IRB 2600 base son un port de la implementación original en ROS 1 Noetic creada por [RAMEL-ESPOL/IRB2600-ABB](https://github.com/RAMEL-ESPOL/IRB2600-ABB) (© 2022, Francisco Yumbla, Javier Pagalo).
