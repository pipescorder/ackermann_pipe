# Robot Ackermann

## Descripción del proyecto

Este paquete ROS2, **ackermann_pipe**, implementa la simulación y visualización de un robot móvil con cinemática Ackermann y un sensor LIDAR en ROS2 Iron (Ubuntu 22.04). El objetivo es cumplir con una estructura en el código de manera modular con URDF, Xacro, nodos de ROS2, visualización en RViz y simulación en Gazebo.

**Autor:** Gabriel Escobar, Benjamín Pérez y Ignacio Parra
**Versión:** 0.1.0

---

## Índice

1. [Requisitos de Software](#requisitos-del-software)
2. [Instalación y compilación](#instalación-y-compilación)
3. [Visualización en RViz](#visualización-en-rviz)
4. [Simulación en Gazebo](#simulación-en-gazebo)
5. [Explicación de package.xml y setup.py](#explicación-de-packagexml-y-setuppy)
6. [Modificación de colores](#modificación-de-colores)
7. [Control Teleop](#control-teleop)
8. [Estructura de carpetas](#estructura-de-carpetas)
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
* Opcional: `teleop_twist_keyboard` para control con teclado del robot

## Instalación y compilación


- ROS2 [ROS2 Documentation](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) (Seguir Desktop Install)
- Gazebo ```sudo apt install ros-iron-gazebo-ros-pkgs```
- Xacro ```sudo apt install ros-iron-xacro```
- `iron`: Actualmente se está empleando **iron**

El archivo URDF (Unified Robot Description Format) contiene la descripción estructural del robot móvil tipo Ackermann. Define los enlaces (`links`) y articulaciones (`joints`) que componen su cuerpo, incluyendo sus dimensiones, jerarquía cinemática y propiedades visuales y físicas necesarias para su simulación. Este se debe ejecutar de la siguiente forma:

- [ackermann urdf](https://github.com/pipescorder/ackermann_pipe/blob/main/urdf/ackermann_model.urdf)
  
* **Comandos de ejecución**

```bash

ros2 launch urdf_tutorial display.launch.py model:=<path to ackermann_model.urdf>
```
Reemplaza <path to ackermann_model.urdf> con la ruta absoluta o relativa al archivo dentro de tu espacio de trabajo.

> ⚠️ **IMPORTANTE:** Asegurar la instalacion del paquete `urdf_tutorial` antes de lanzar el archivo URDF con `sudo apt install ros-iron-urdf-tutorial`.

> 📝 **Nota:** Una vez hecho lo anterior, se inicia con la ejecución de las simulaciones


1. **Configurar el entorno ROS2** (en cada terminal nueva):
```bash
   source /opt/ros/iron/setup.bash
 ```
2. **Situarse en este paquete** dentro del workspace ROS2.
```bash
   cd ~/ackermann_pipe
 ```
3. **Compila con colcon** desde la raíz del workspace ackermann_pipe:
```bash
   colcon build
 ```
4. **Fuente del entorno de instalación**:
```bash
   source install/local_setup.bash
 ```

## Visualización en RViz

El archivo de lanzamiento en python es creado en un paquete de ROS. Para más información sobre la creación de paquetes y su compilación ver [creating a package](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Para la visualización mediante los nodos `robot_state_publisher`, `joint_state_publisher` y `rviz2` se realizó el siguiente launch file [visualization.launch.py](https://github.com/pipescorder/ackermann_pipe/blob/main/launch/visualization.launch.py) y se ejecuta de la siguiente manera:

1. **Lanzar el nodo de visualización**:
   Una vez realizado anteriormente  `colcon build` y `source install/local_setup.bash`, se procede a:
```bash
   ros2 launch ackermann_pipe visualization.launch.py
 ```
2. **Control de articulaciones**:

   * Usar la ventana de `joint_state_publisher_gui` para mover ruedas o ejes de dirección.

3. **rqt_graph**:
```bash
   ros2 run rqt_graph rqt_graph
 ```

   * Verificar nodos `/robot_state_publisher`, `/joint_state_publisher_gui`, tópicos `/joint_states`, `/tf`.

## Simulación en Gazebo

1. **Lanzar simulación completa**:
```bash
   ros2 launch ackermann_pipe gazebo.launch.py
 ```
3. **Mover robot**:

   * Abre un nuevo terminal y lanza teleop:
```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
 ```
   * Usar `i/j/k/l` para avanzar, girar, retroceder.

   
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

## Control Teleop

* **Instalación**:
```bash
  sudo apt install ros-iron-teleop-twist-keyboard
```
* **Ejecución**:
```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

 
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
`````
## Consejos de depuración

* **Warnings de material**:

  * Si ves `material 'blue' undefined`, revisa `lidar.xacro` y reemplaza `blue` por `greenX`.
* **Gazebo no inicia**:

  ```bash
  pkill gzserver && pkill gzclient
  ```
* **Archivos no encontrados**:

  * Asegúrarse de usar `FindPackageShare('ackermann_pipe')` en los launch para rutas dinámicas.
