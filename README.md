# ackermann\_pipe

## Descripción del proyecto

Este paquete ROS2, **ackermann\_pipe**, implementa la simulación y visualización de un robot móvil con cinemática Ackermann y un sensor LIDAR en ROS2 Iron (Ubuntu 22.04). El objetivo es cumplir con los requisitos del proyecto de simulación especificados en el documento PDF, estructurando el código de manera modular con URDF, Xacro, nodos de ROS2, visualización en RViz y simulación en Gazebo.

**Autor:** Gabriel Escobar
**Versión:** 0.1.0

---

## Índice

1. [Estructura de carpetas](#estructura-de-carpetas)
2. [Requisitos de software](#requisitos-de-software)
3. [Instalación y compilación](#instalación-y-compilación)
4. [Descripción de los archivos principales](#descripción-de-los-archivos-principales)
5. [Uso de URDF y Xacro](#uso-de-urdf-y-xacro)
6. [Visualización en RViz](#visualización-en-rviz)
7. [Simulación en Gazebo](#simulación-en-gazebo)
8. [Control Teleop](#control-teleop)
9. [Modificación de colores](#modificación-de-colores)
10. [Explicación de package.xml y setup.py](#explicación-de-packagexml-y-setuppy)
11. [Consejos de depuración](#consejos-de-depuración)

---


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
 
  * 
## Descripción de los archivos principales

* \`\`: Lanza RViz, `robot_state_publisher` y `joint_state_publisher_gui` para visualizar el robot y mover articulaciones.
* \`\`: Lanza Gazebo, publica el modelo URDF, spawnea la entidad y ejecuta RViz simultáneamente.
* \`\`: Xacro principal que incluye todos los módulos (`params.xacro`, `ackermann.xacro`, `gazebo.xacro`, `lidar.xacro`).
* \`\`: Define propiedades reutilizables (dimensiones, radios, colores, etc.).
* \`\`: Contiene la macro de chasis y ruedas.
* \`\`: Añade plugins de Gazebo para cinemática Ackermann y sensor.
* \`\`: Define el sensor LIDAR y su driver `libgazebo_ros_ray_sensor.so`.
* \`\`: Archivo de configuración de RViz para mostrar el robot y el mensaje `LaserScan`.
* \`\`: Manifiesto del paquete ROS2 con dependencias y metadatos.
* \`\`: Script de instalación Python que incluye archivos de datos (launch, urdf, rviz, imgs).



## Instalación y compilación

1. **Configura tu entorno ROS2** (en cada terminal nueva):

   ```bash
   source /opt/ros/iron/setup.bash
   ```
2. **Clona o sitúa este paquete** dentro de tu workspace ROS2 (`src/`).

   ```bash
   cd ~/ackermann_pipe
   ```
3. **Compila con colcon** desde la raíz del workspace:

   ```bash
   colcon build --symlink-install --cmake-clean-cache
   ```
4. **Fuente del entorno de instalación**:

   ```bash
   source install/local_setup.bash
   ```

> Tras estos pasos, tendrás los nodos, URDF y Xacro instalados en `install/share/ackermann_pipe`.

---

## Uso de URDF y Xacro

1. **¿Qué es URDF?**

   * Formato XML para describir links, joints, geometría y materiales.
2. **¿Por qué Xacro?**

   * Permite modularizar, parametrizar y simplificar la generación de URDF.
3. **Generar URDF a partir de Xacro**:

   ```bash
   xacro `ros2 pkg prefix ackermann_pipe`/share/ackermann_pipe/urdf/robot.xacro > robot.urdf
   ```
4. **Verificar sintaxis**:

   ```bash
   check_urdf robot.urdf
   ```

   
## Visualización en RViz

1. **Lanzar el nodo de visualización**:

   ```bash
   ros2 launch ackermann_pipe visualization.launch.py
   ```
2. **Control de articulaciones**:

   * Usa la ventana de `joint_state_publisher_gui` para mover ruedas o ejes de dirección.
3. **rqt\_graph**:

   ```bash
   ros2 run rqt_graph rqt_graph
   ```

   * Verifica nodos `/robot_state_publisher`, `/joint_state_publisher_gui`, tópicos `/joint_states`, `/tf`.

---

## Control Teleop

* **Instalación**:

  ```bash
  sudo apt install ros-iron-teleop-twist-keyboard
  ```
* **Ejecución**:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
* **Tópico**: publica en `/cmd_vel`; el plugin de Gazebo lo suscribe para mover el robot.


## Simulación en Gazebo

1. **Lanzar simulación completa**:

   ```bash
   ros2 launch ackermann_pipe gazebo.launch.py
   ```
2. **Verificar spawn**:

   * Observa mensajes en consola: `spawn_entity`: `entity robot`.
3. **Mover robot**:

   * Abre un nuevo terminal y lanza teleop:

     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```
   * Usa `i/j/k/l` para avanzar, girar, retroceder.
   * 

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
* 
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



