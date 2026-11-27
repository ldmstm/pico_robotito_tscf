# Nav2 Quickstart

## Requisitos

Una vez que se tenga armado el contenedor de Docker, el firmware este flasheado en el robot, y zenohd/sync_time esten ejecutando
se deben iniciar los siguientes scripts:

### robotito_description
Desde `pico-robotito/robotito_description`:
```colcon build```

### URDF -> TF
Desde `pico-robotito/robotito_description/src/description`:
```ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ./robotito_description.urdf.xacro)"```

### JSP
```ros2 run joint_state_publisher joint_state_publisher```

### /odom -> base_footprint
Desde `pico-robotito/robotito_description`:
```python3 launch/odom_tf_relay.py```

### /sensor_N_range -> /scan
Desde `pico-robotito/robotito_description`:
```python3 launch/ranges_to_pointcloud.py```

### SLAM
Desde `pico-robotito/robotito_description`:
```ros2 launch slam_toolbox online_async_launch.py slam_params_file:=config/slam_params.yaml use_sim_time:=false```

### Nav2
Desde `pico-robotito/robotito_description`:
```ros2 launch nav2_bringup navigation_launch.py params_file:=config/nav2_params.yaml use_sim_time:=false```

### RViz2
```rviz2```

#### Configuracion de RViz2
Global opts -> Fixed frame := Map (seteado asi por default en Jazzy)

Se deben agregar:
 - RobotModel (con Topic Description `/robot_description`)
 - Map (/map)
 - Map (/global_costmap/map)
 - Map (/local_costmap/map)
 - LaserScan (/scan)
 - Path (/plan)

En este momento ya se puede ver el resultado del LaserScan y los mapas de costos,
y se puede usar `2D Goal Pose` para indicarle a Robotito hacia donde debe moverse.

Tambien se puede mover al robot manualmente y la visualizacion se actualiza sola (por `/odom`).
