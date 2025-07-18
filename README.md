# Ackermann_pipe

## Descripción del proyecto

Este paquete ROS2, **ackermann_pipe**, implementa la simulación y visualización de un robot móvil con cinemática Ackermann y un sensor LIDAR en ROS2 Iron (Ubuntu 22.04). El objetivo es cumplir con los requisitos del proyecto de simulación especificados en el documento PDF, estructurando el código de manera modular con URDF, Xacro, nodos de ROS2, visualización en RViz y simulación en Gazebo.

**Autor:** Gabriel Escobar
**Versión:** 0.1.0

---

## Índice

1. [Estructura de carpetas](#estructura-de-carpetas)
2. [Requisitos de Software](#requisitos-del-software)
3. [Instalación y compilación](#instalación-y-compilación)
4. [Visualización en RViz](#visualización-en-rviz)
5. [Simulación en Gazebo](#simulación-en-gazebo)
6. [Control Teleop](#control-teleop)
7. [Modificación de colores](#modificación-de-colores)
8. [Explicación de package.xml y setup.py](#explicación-de-packagexml-y-setuppy)
9. [Consejos de depuración](#consejos-de-depuración)

---

## Requisitos del Software

* **Sistema operativo:** Ubuntu 22.04 LTS
* **ROS2 Iron** instalado (desktop)
* Paquetes ROS2 necesarios:

  * `xacro`
  * `robot_state_publisher`
  * `joint_state_publisher_gui`
  * `gazebo_ros`
  * `rviz2`
* **Python 3.10+** con `rclpy`
* Opcional: `teleop_twist_keyboard` para control con teclado

## Explicación de package.xml y setup.py

* **package.xml**: declara metadatos, dependencias y exporta `ament_python`.
* **setup.py**: indica a setuptools qué paquetes y archivos de datos instalar en `install/share/ackermann_pipe`.

  * Se incluyen los directorios `launch/`, `urdf/`, `rviz/`, `imgs/`.


## Modificación de colores

Todas las apariencias se definen en `urdf/params.xacro` y `<material>` en `robot.xacro`:

```xml
<xacro:property name="green1_rgba" value="0.6 1.0 0.6 1.0"/>
<material name="green1"><color rgba="${green1_rgba}"/></material>
```

Para cambiar tono:

1. Edita valores RGBA en `params.xacro`.
2. Reconstruye y relanza RViz/Gazebo:

   ```bash
   colcon build && source install/local_setup.bash
   ros2 launch ackermann_pipe visualization.launch.py
   ```

 
## Consejos de depuración

* **Warnings de material**:

  * Si ves `material 'blue' undefined`, revisa `lidar.xacro` y reemplaza `blue` por `greenX`.
* **Gazebo no inicia**:

  ```bash
  pkill gzserver && pkill gzclient
  ```
* **Errores de transformaciones**:

  * Revisa que todos los `link` estén unidos mediante `joint` y publicados por `robot_state_publisher`.
* **Archivos no encontrados**:

  * Asegúrate de usar `FindPackageShare('ackermann_pipe')` en los launch para rutas dinámicas.
 
## Instalación y compilación

1. **Configura tu entorno ROS2** (en cada terminal nueva):

   source /opt/ros/iron/setup.bash
 
2. **Clona o sitúa este paquete** dentro de tu workspace ROS2 (`src/`).

   cd ~/ackermann_pipe

3. **Compila con colcon** desde la raíz del workspace:

   colcon build --symlink-install --cmake-clean-cache

4. **Fuente del entorno de instalación**:

   source install/local_setup.bash
 
   
## Visualización en RViz

1. **Lanzar el nodo de visualización**:

   ros2 launch ackermann_pipe visualization.launch.py

2. **Control de articulaciones**:

   * Usar la ventana de `joint_state_publisher_gui` para mover ruedas o ejes de dirección.

3. **rqt_graph**:

   ros2 run rqt_graph rqt_graph


   * Verificar nodos `/robot_state_publisher`, `/joint_state_publisher_gui`, tópicos `/joint_states`, `/tf`.


## Control Teleop

* **Instalación**:

  sudo apt install ros-iron-teleop-twist-keyboard

* **Ejecución**:

  ros2 run teleop_twist_keyboard teleop_twist_keyboard


## Simulación en Gazebo

1. **Lanzar simulación completa**:

   ros2 launch ackermann_pipe gazebo.launch.py

3. **Mover robot**:

   * Abre un nuevo terminal y lanza teleop:

     ros2 run teleop_twist_keyboard teleop_twist_keyboard

   * Usar `i/j/k/l` para avanzar, girar, retroceder.
   

## Estructura de carpetas

`````bash
ackermann_pipe/                # Raíz del paquete
├── ackermann_pipe/            # Código Python (__init__.py)
│   └── __init__.py            # Package Python
├── build/                     # Carpeta generada por colcon build
├── install/                   # Carpeta de instalación generada por colcon build
├── launch/                    # Archivos de lanzamiento (.launch.py)
│   ├── visualization.launch.py
│   └── gazebo.launch.py
├── log/                       # Registros de ejecución (ros2, colcon)
├── resource/                  # Índice de recursos ROS2
│   └── ackermann_pipe
├── rviz/                      # Configuración de RViz
│   └── config.rviz
│   └── config_lidar.rviz
├── test/                      # Pruebas unitarias (pytest)
├── urdf/                      # Modelos URDF y Xacro
│   ├── robot.xacro
│   ├── params.xacro
│   ├── ackermann.xacro
│   ├── ackermann_model.urdf
│   ├── ackermann_sensor.urdf
│   ├── ackermann_sensor_xacro.urdf
│   ├── gazebo.xacro
│   └── lidar.xacro
├── world/                     # Definiciones de mundos de Gazebo
├── package.xml                # Manifiesto ROS2
├── setup.py                   # Script de instalación Python
├── setup.cfg                  # Configuración de linting/tests
└── README.md                  # Documentación del proyecto
