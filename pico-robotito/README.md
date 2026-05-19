# ejemplo-pros-echo — Nodo de eco WiFi con Pico-ROS

> **Nota:** Este documento fue generado con asistencia de IA.

Firmware mínimo para el **Sparkfun ESP32 Thing** que implementa un nodo ROS 2
de eco por WiFi usando [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software),
compilado con PlatformIO + ESP-IDF.  Todo el código está escrito en **C puro**
utilizando la API C de Pico-ROS (`picoros.h` / `picoserdes.h`).

El nodo se suscribe al tópico `echo_in` (`std_msgs/String`) y republica
cada mensaje recibido en el tópico `echo_out`.

---

## Funcionamiento

### Arquitectura

```
Host ROS 2                              ESP32

 ros2 topic pub /pico_echo/echo_in       ┌──────────────┐
     std_msgs/String "Hola"   ──TCP──►   │  suscriptor   │
                                         │  echo_in      │
                                         │       │       │
                                         │   callback    │
                                         │   deserializar│
                                         │   serializar  │
                                         │       │       │
                                         │  publicador   │
 ros2 topic echo /pico_echo/echo_out     │  echo_out     │
     "Hola"                   ◄──TCP──   └──────────────┘
```

### Secuencia de arranque

1. **Inicialización de NVS** — El almacenamiento no volátil se inicializa
   (requerido por el driver WiFi de ESP-IDF).
2. **Conexión WiFi** — El ESP32 se conecta como estación (STA) al punto de
   acceso configurado.  El firmware bloquea hasta obtener dirección IP.
3. **Interfaz Zenoh** — Se abre una sesión zenoh-pico en **modo client**
   conectada al router Zenoh vía TCP.
4. **Nodo Pico-ROS** — Se crea el nodo (`pico_echo`) con el domain ID
   especificado, mediante `picoros_node_init()`.
5. **Publicador** — Se declara un publicador en el tópico `echo_out`
   (`std_msgs/String`) con `picoros_publisher_declare()`.
6. **Suscriptor** — Se declara un suscriptor en el tópico `echo_in`
   (`std_msgs/String`) con `picoros_subscriber_declare()` y se registra
   la función `echo_callback`.
7. **Bucle principal** — La tarea principal queda en idle (`vTaskDelay`).
   Todo el trabajo ocurre en el callback del suscriptor, invocado por Zenoh
   al recibir un mensaje.

### Callback (`echo_callback`)

Cuando llega un mensaje en `echo_in`:

1. El payload CDR se **deserializa** a un string C con `ps_deserialize()`.
2. El string se registra en el monitor serie vía `ESP_LOGI`.
3. El mismo string se **re-serializa** a formato CDR con `ps_serialize()`.
4. El payload se **publica** en `echo_out` mediante `picoros_publish()`.

---

## Estructura del proyecto

```
ejemplo-pros-echo/
├── CMakeLists.txt              # CMake raíz (boilerplate ESP-IDF)
├── platformio.ini              # Configuración de compilación PlatformIO
├── .clangd                     # Configuración de clangd (LSP)
├── .gitignore
├── config/
│   └── ucdr/
│       └── config.h            # Endianness para Micro-CDR
├── include/
│   └── my_ros_types.h          # Definiciones CDR (std_msgs/String, etc.)
├── src/
│   ├── CMakeLists.txt          # Registro de fuentes del componente
│   ├── main.c                  # Punto de entrada del nodo echo
│   ├── wifi_connection.h       # Auxiliar WiFi STA (header)
│   └── wifi_connection.c       # Auxiliar WiFi STA (implementación)
├── lib/                        # (vacío — para bibliotecas locales)
└── test/                       # (vacío — para tests unitarios)
```

---

## Requisitos previos

- Un **router Zenoh** corriendo en la red, accesible desde el ESP32 por TCP.
  Con ROS 2 Jazzy y `rmw_zenoh`:
  ```bash
  ros2 run rmw_zenoh_cpp rmw_zenohd
  ```
  Los archivos de configuración del router y de la sesión Zenoh se encuentran en:
  ```
  /opt/ros/jazzy/share/rmw_zenoh_cpp/config/
  ```
  Ahí se encuentran `DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5` y
  `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`.  Para usar una copia personalizada:
  ```bash
  export ZENOH_ROUTER_CONFIG_URI=$HOME/mi_router_config.json5
  ros2 run rmw_zenoh_cpp rmw_zenohd
  ```
- ROS 2 Jazzy configurado con `rmw_zenoh`:
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
- **Pico-ROS-software** clonado manualmente (ver sección siguiente).

---

## Instalación de dependencias

`zenoh-pico` y `Micro-CDR` se descargan automáticamente por PlatformIO.
**Pico-ROS-software** debe clonarse a mano:

```bash
cd ejemplo-pros-echo
# Compilar una vez para que PlatformIO cree la carpeta de dependencias:
pio run -e esp32thing || true

# Clonar Pico-ROS en la carpeta de librerías del entorno:
cd .pio/libdeps/esp32thing
git clone https://github.com/Pico-ROS/Pico-ROS-software.git
cd ../../..
```

| Biblioteca | Propósito |
|---|---|
| [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico) | Cliente Zenoh para sistemas embebidos |
| [Micro-CDR](https://github.com/eProsima/Micro-CDR) | Serialización CDR (formato DDS) |
| [Pico-ROS-software](https://github.com/Pico-ROS/Pico-ROS-software) | API C de Pico-ROS (`picoros.h`, `picoserdes.h`) |

---

## Configuración

Editar el bloque `#define` en [src/main.c](src/main.c):

| Define | Descripción | Ejemplo |
|---|---|---|
| `WIFI_SSID` | Nombre de la red WiFi | `"mi_red"` |
| `WIFI_PASSWORD` | Contraseña WiFi | `"mi_clave"` |
| `ZENOH_ROUTER_ADDRESS` | Dirección TCP del router Zenoh | `"tcp/192.168.1.100:7447"` |
| `ZENOH_NODE_NAME` | Nombre del nodo ROS | `"pico_echo"` |
| `ROS_DOMAIN_ID` | Domain ID de ROS 2 (debe coincidir con el host) | `0` |

---

## Compilar y flashear

```bash
cd ejemplo-pros-echo
pio run -e esp32thing
pio run -e esp32thing -t upload
pio device monitor
```

---

## Prueba con ROS 2

Desde el host ROS 2 (con `rmw_zenoh` y el router Zenoh corriendo):

```bash
# Terminal 1: escuchar los mensajes de eco
ros2 topic echo /pico_echo/echo_out std_msgs/msg/String

# Terminal 2: enviar un mensaje
ros2 topic pub --once /pico_echo/echo_in std_msgs/msg/String '{data: "Hola desde ROS 2!"}'
```

Salida esperada en Terminal 1:

```
data: Hola desde ROS 2!
```

Salida esperada en el monitor serie del ESP32:

```
I (12345) echo: Recibido: Hola desde ROS 2!
I (12346) echo: Eco: Hola desde ROS 2!
```

---

## Definiciones de tipos

El archivo [include/my_ros_types.h](include/my_ros_types.h) contiene las
definiciones CDR que picoserdes usa para serialización/deserialización.
Este ejemplo incluye solamente:

- `ros_Duration` y `ros_Time` — requeridos internamente por Pico-ROS.
- `ros_Log` — usado si se activa el módulo rosout.
- `ros_String` — el tipo `std_msgs/String` usado por los tópicos de eco.

Para proyectos que necesiten más tipos, usar la
[herramienta de generación](https://github.com/Pico-ROS/Pico-ROS-software/tree/master/tools/type-gen)
o copiar `example_types.h` del repositorio Pico-ROS.

---

## Documentación del código

Todo el código fuente está documentado con comentarios **Doxygen**.
Para generar la documentación HTML:

```bash
cd ejemplo-pros-echo
doxygen -g   # genera Doxyfile por defecto (solo la primera vez)
doxygen
```
