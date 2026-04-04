# pico_robotito_tscf

Firmware para [Robotito](https://www.youtube.com/watch?v=Jo6SKr23Z8M), usando ROS 2 (particularmente [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software)).  
**Estado actual:** proyecto en fase inicial; la *launch board* ya ejecuta un ejemplo **Hello World**.

---

## Objetivos del proyecto (pendiente)

---

## Arquitectura / Diseño (pendiente)

---

## Requisitos (pendiente)

---

## Configuración del entorno (pendiente)

---

## Compilar, flashear y monitorizar (PlatformIO)

### Opción A: VS Code (interfaz de PlatformIO)
1. Instala **VS Code**.
2. Instala la extensión **PlatformIO IDE**.
3. Abre esta carpeta como proyecto en VS Code.
4. Usa las acciones típicas de PlatformIO:
   - **Build**
   - **Upload**
   - **Monitor**

> Completar aquí con el nombre de entorno (`env`) y puerto/baud reales cuando estén definidos.

### Opción B: CLI (`pio`)
1. Instala PlatformIO Core (CLI).
2. En la raíz del repo, usa los siguientes comandos (ajusta el `-e` al entorno real; p. ej. `sparkfun_esp32_thing`):

```bash
pio run -t clean

pio run -e sparkfun_esp32_thing -t upload

pio device monitor -b 115200 -p /dev/ttyUSB0
```

Notas:
- El valor de `-e` debe coincidir con un **environment** definido en `platformio.ini` (por ejemplo `sparkfun_esp32_thing`, en lugar de `esp32dev`).
- El puerto serie suele ser `/dev/ttyUSB0` en Linux, pero puede variar (p. ej. `/dev/ttyUSB1`, `/dev/ttyACM0`, etc.).

---

## Uso (pendiente)

---

## ROS 2 / Integración (pendiente)

---

## Pruebas (pendiente)

---

## Roadmap (pendiente)

---

## Errores Conocidos

### Desajuste de frecuencia del cristal (XTAL) entre host y dispositivo
**Síntoma:** fallos al comunicar/parsear cuando el host asume una frecuencia de XTAL distinta a la del dispositivo.

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
