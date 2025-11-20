# lawn_robot

Paquete ROS 2 que contiene el modelo URDF/Xacro de un robot diferencial tipo podadora con cuatro ruedas y tres sensores ultrasónicos listos para simulacion en Gazebo.

## Requerimientos
- ROS 2 con `gazebo_ros`, `xacro`, `robot_state_publisher` y `joint_state_publisher` instalados (`sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro`).
- `colcon` para construir el workspace.
- Gazebo Classic (parte de `gazebo_ros`) y opcionalmente `ros_gz_bridge` si se piensa usar Ignition/Garden.

## Estructura rapida
- `model/robot.xacro`: definicion principal del robot (cuerpo, ruedas, sensores).
- `model/ultrasounds.xacro`: tres sensores tipo ray configurados como LaserScan.
- `model/robot.gazebo`: parametros de friccion y plugin `gazebo_ros_diff_drive` (usa `/cmd_vel` y publica odom/TFs).
- `model/ros2_control.xacro`: punto de partida para integrar ros2_control (no incluido en el launch actual).
- `model/robot.urdf`: URDF generado desde el Xacro.
- `meshes/base_link.stl`: malla visual del cuerpo.
- `config/*.rviz`: configuraciones listas para RViz2.
- `launch/gazebo_model.launch.py`: lanza Gazebo, publica el robot_description y hace spawn del modelo.

## Construir
```bash
cd ~/lawn_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select lawn_robot
source install/setup.bash
```

## Generar el URDF desde Xacro
```bash
cd ~/lawn_ws/src/lawn_robot/model
ros2 run xacro xacro robot.xacro -o robot.urdf
```

## Lanzar en Gazebo
```bash
source ~/lawn_ws/install/setup.bash
ros2 launch lawn_robot gazebo_model.launch.py
```

Notas:
- El launch intenta abrir `model/empty_world.world`. Si no existe, actualiza la ruta en `gazebo_model.launch.py` o coloca tu mundo en `model/`.
- El plugin dif-drive escucha `/cmd_vel`, publica `/odom` y los TF de las ruedas.

## Visualizar en RViz2
Con Gazebo corriendo y el robot publicado:
```bash
rviz2 -d $(ros2 pkg prefix lawn_robot)/share/lawn_robot/config/view_lawn.rviz
```

## Teleoperar rapido
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## Pendiente
- Definir licencia y descripcion en `package.xml`.
- Agregar mundo de ejemplo o permitir usar el `empty.world` de Gazebo por defecto.
- Conectar `ros2_control` si se desea controladores en lugar del plugin de Gazebo.
