# pico_robotito_tscf

Firmware para [Robotito](https://robotito.pages.fing.edu.uy/firmware/about/), un robot educativo originalmente programado con una API pequeña en Lua, ahora refactorizado a **C** para poder integrarse con **ROS 2 Jazzy** mediante **Zenoh** (particularmente [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)) y [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software/tree/master).

Este repositorio contiene el firmware y el flujo de trabajo del host usados para validar la comunicación ROS 2 con el robot. El alcance actual incluye conectividad WiFi, publicación y suscripción de tópicos ROS 2, sincronización de hora, teleoperación por teclado y visualización de odometría con RViz.

El pfue desarrollado para la placa **SparkFun ESP32 Thing** usando **PlatformIO** con framework **ESP-IDF**.

Actualmente el robot:
- se conecta por WiFi,
- se comunica con ROS 2 mediante Zenoh,
- expone nodos y tópicos ROS 2,
- soporta los tópicos documentados en [`ros2_topics_cheatsheet.md`](ros2_topics_cheatsheet.md).

Los tópicos validados actualmente son:
- `/cmd_vel`
- `/joint_states`
- `/odom`
- `/tf`
- `/tick`
- `/rosout`
- `/sync_time_request`
- `/sync_time_response`

Referencias:
- Robotito: https://robotito.pages.fing.edu.uy/firmware/about/
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/index.html
- Pico-ROS: https://github.com/Pico-ROS/Pico-ROS-software
- Repositorio base de comunicación: `xopxe/picorosso-espidf`
- Fork usado para la migración a C: `martinrugnitz/picorosso-espidf`, branch `c-migration`


## Dependencias y datos del entorno

Entorno usado para las pruebas:
- **SO / distro:** Artix Linux `7.1.2-artix3-1.1`
- **Sistema de ventanas:** X11
- **Editor / contenedores:** VS Code + Dev Containers
- **Distro ROS 2:** Jazzy Jalisco
- **Microcontrolador:** SparkFun ESP32 Thing
- **Framework:** PlatformIO con **ESP-IDF**

Software requerido:
- Docker
- VS Code + Dev Containers o Docker CLI
- Acceso al dispositivo serie del ESP32 desde el host
- Red WiFi 2.4 GHz

Repositorios externos:
- `xopxe/picorosso-espidf`: base de la interfaz de comunicación
- `martinrugnitz/picorosso-espidf` (branch `c-migration`): migración a C para usar `zenoh-pico` y `Pico-ROS`
- https://github.com/xopxe/ros2_sync_time_service_ws/tree/main: servicio de sincronización de hora
- https://github.com/Pico-ROS/Pico-ROS-software/tree/master: dependencia manual de PlatformIO
- Documentación oficial de ROS2: https://docs.ros.org/en/jazzy/index.html

---

## Configuración, compilación y ejecución

### 1. Clonar el repositorio

```bash
git clone https://github.com/martinrugnitz/pico_robotito_tscf.git
cd pico_robotito_tscf
```

### 2. Buildear y correr el contenedor

#### Opción A: VS Code + Dev Containers

Abrir el repositorio en VS Code y utilizar la extension Dev Containers para abrir en el container especificado en la carpeta `.devcontainers`.

#### Opción B: Docker CLI

Desde la raíz del repo:

```bash
docker build -t pico_robotito_tscf-dev -f .devcontainer/Dockerfile .devcontainer
```

Ejecutar el contenedor con acceso a dispositivos, red de host y X11:

```bash
docker run --rm -it \
  --privileged \
  --network=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "$(pwd)":/workspace/pico_robotito_tscf \
  -w /workspace/pico_robotito_tscf \
  pico_robotito_tscf-dev
```

Si se sugiere reutilizar el estado local de PlatformIO:

```bash
-v "$HOME/.platformio:/home/ubuntu/.platformio"
```

### 3. Configurar el firmware

Editar en `pico-robotito/src/main.c`:

```c
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PSWRD"
#define ZENOH_ROUTER_ADDRESS "tcp/xxx.xxx.xxx.xxx:7447"
```

Notas:
- El ESP32 solo soporta redes 2.4 GHz.
- La IP debe ser la del host donde corre el router Zenoh.
- Estos valores deben revisarse antes de cada flasheo.

### 4. Bootstrap de dependencias de PlatformIO

Desde el directorio del firmware:

```bash
cd /workspace/pico_robotito_tscf/pico-robotito
pio run -e esp32thing
```

Eso crea el árbol de dependencias de PlatformIO. Luego instalar **Pico-ROS-software** manualmente en `.pio/libdeps/esp32thing`:

```bash
cd .pio/libdeps/esp32thing
git clone https://github.com/Pico-ROS/Pico-ROS-software.git
cd Pico-ROS-software
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```

Si `git submodule update` pide credenciales SSH, hacer el clone y el update desde el host y dejar el build para el contenedor.

### 5. Compilar, flashear y monitorizar

```bash
cd /workspace/pico_robotito_tscf/pico-robotito

pio run -e esp32thing
pio run -e esp32thing -t upload
pio device monitor -b 115200
```

También puede usarse la interfaz gráfica de PlatformIO en VS Code.

Si el upload falla, revisar:
- conexión de la placa,
- acceso a `/dev` desde el contenedor,
- nombre correcto del puerto serie,
- board correcta en PlatformIO.

### 6. Levantar ROS 2 del lado host/contenedor

Sourcear ROS 2 si hace falta:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Levantar el router Zenoh:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

En otra terminal, levantar el servicio de `sync_time`:
```bash
ros2 run sync_time sync_time_node
```

Si no quedó sourceado automáticamente:

```bash
source /home/ubuntu/ros2_sync_time_service_ws/install/setup.bash
```

Luego encender el robot y verificar descubrimiento:

```bash
ros2 node list
ros2 topic list
ros2 topic list -t
```

### 7. Teleoperación y visualización

Mover el robot desde teclado:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Visualizar odometría en RViz:

```bash
ros2 run rviz2 rviz2
```

Para que `rviz2` corra dentro del contenedor, es necesario forwardear los puertos de X11:

```bash
xhost +local:docker
xhost +local:
```

Y dentro del contenedor conviene verificar que las siguientes variables sean las mismas que en el host:

```bash
echo $DISPLAY
echo $XDG_RUNTIME_DIR
ls /tmp/.X11-unix
```

### 8. Stack Nav2

Ver `nav2/nav2_quickstart.md` por mas informacion.

---

## Problemas conocidos y soluciones

### 1. Desajuste de frecuencia del cristal (XTAL) entre host y dispositivo

**Síntoma:** fallos al comunicar o parsear cuando el host asume una frecuencia de XTAL distinta a la del dispositivo.

**Causa:** el host estaba configurado a **40 MHz**, pero el dispositivo usa **26 MHz**.

**Cambio aplicado (en `sdkconfig.sparkfun_esp32_thing`):**
```ini
#
# Main XTAL Config
#
CONFIG_XTAL_FREQ_26=y
# CONFIG_XTAL_FREQ_32 is not set
# CONFIG_XTAL_FREQ_40 is not set
# CONFIG_XTAL_FREQ_AUTO is not set
CONFIG_XTAL_FREQ=26
# end of Main XTAL Config
```

### 2. Pico-ROS no queda instalado automáticamente

**Problema:** `Pico-ROS-software` no se descarga ni compila solo.

**Solución:** ejecutar el bootstrap de PlatformIO, clonar manualmente el repo en `.pio/libdeps/esp32thing`, inicializar submódulos y compilarlo con `cmake` + `make`.

### 3. RViz2 dentro del contenedor no abre

**Problema:** `ros2 run rviz2 rviz2` no muestra interfaz o falla al arrancar.

RViz2 es un programa que corre sobre X11, si el sistema nativamente tiene un servidor ejecutando
se puede usar la variable de entorno `DISPLAY` para desplegarlo en el escritorio nativo.

Para sistemas que no poseen un servidor nativo (Windows + WSL, Linux con Wayland, etc) se preinstala
un servidor VNC y una instalacion minima de icewm. En el entorno de Docker se debe configurar una password
para el servidor (mediante `vncpasswd`) y se debe iniciar el servidor (`vncserver :99`).
Desde el host se puede conectar a `localhost:5999` para poder interactuar con el servidor VNC.
