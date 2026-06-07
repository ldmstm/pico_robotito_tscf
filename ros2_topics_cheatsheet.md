# ROS 2 Cheatsheet: Topicos

Comandos disponibles:

```bash
ros2 topic list -t
ros2 topic type <topic>
ros2 topic info <topic>
```

## `/cmd_vel`

Type: `geometry_msgs/msg/TwistStamped`

Rol: subscriptor en el robot. Subale mambo para que mis gatas prendan los motore (i.e. accionar los motores).

`ros2 topic info`:

```text
Type: geometry_msgs/msg/TwistStamped
Publisher count: 0
Subscription count: 1
```

Ejemplos:

```bash
# Moverse para "adelante" continuamente (`-r 10` == publicar a 10Hz)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# Rotar sobre su propio eje una vez (--once)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}}"
```

## `/joint_states`

Type: `sensor_msgs/msg/JointState`

Role: publicador en el robot. Info de ruedas (odometria).

`ros2 topic info`:

```text
Type: sensor_msgs/msg/JointState
Publisher count: 1
Subscription count: 0
```

Ejemplos:

```bash
# Imprime los mensajes que llegan
ros2 topic echo /joint_states

# Mide la frecuencia de arribo de mensajes
ros2 topic hz /joint_states
```

## `/odom`

Type: `nav_msgs/msg/Odometry`

Role: publicado por el robot. Estimaciones de posicion y velocidad (odometria).

`ros2 topic info`:

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
```

Ejemplos:

```bash
ros2 topic echo /odom
ros2 topic hz /odom
```

## `/tf`

Type: `tf2_msgs/msg/TFMessage`

Role: publicado por el robot. Transformadas geometricas para la odometria.

`ros2 topic info`:

```text
ros2 topic info /tf
Unknown topic '/tf'
```

Ejemplos:

```bash
ros2 topic echo /tf
```

## `/tick`

Type: `std_msgs/msg/Int32`

Role: Publicado por el robot. Ticker estilo keep alive.

`ros2 topic info` snapshot:

```text
ros2 topic info /tick
Unknown topic '/tick'
```

Ejemplos:

```bash
ros2 topic echo /tick
ros2 topic hz /tick
```

## `/rosout`

Type: `rcl_interfaces/msg/Log`

Role: publicado por el robot y otros nodos ROS2. Es un tick un poco mas fancy.

`ros2 topic info` snapshot:

```text
Type: rcl_interfaces/msg/Log
Publisher count: 3
Subscription count: 0
```

Ejemplos:

```bash
ros2 topic echo /rosout
ros2 topic echo /rosout --once
```

## `/sync_time_request`

Type: `std_msgs/msg/String`

Role: publicado por el robot, subscriptor en el nodo sync-time. Usado para la sincronizacion de hora real con el robot.

`ros2 topic info` snapshot:

```text
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

Ejemplos:

```bash
ros2 topic echo /sync_time_request
ros2 topic hz /sync_time_request
```

## `/sync_time_response`

Type: `sensor_msgs/msg/TimeReference`

Role: subscriptor en el robot, publicado por el nodo sync-time. Usado para la sincronizacion de hora real con el robot.

`ros2 topic info` snapshot:

```text
Type: sensor_msgs/msg/TimeReference
Publisher count: 1
Subscription count: 1
```

Ejemplos:

```bash
ros2 topic echo /sync_time_response
ros2 topic hz /sync_time_response
```