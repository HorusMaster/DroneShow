# Drone Show

Quadcopter ESP32 con telemetría WebSocket y control desde el navegador.

## Compilar y flashear (ESP-IDF)

### 1. Activar el entorno de ESP-IDF

En cada terminal nueva, antes de usar `idf.py`, hay que cargar el entorno:

```bash
. $HOME/esp/esp-idf/export.sh
```

Si sale `idf.py: command not found`, es porque este paso falta en esa terminal.

Atajo opcional — añadir un alias permanente a `~/.bashrc`:

```bash
echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> ~/.bashrc
source ~/.bashrc
```

Después basta con escribir `get_idf` en cualquier terminal.

### 2. Compilar, flashear y monitorear

Desde la raíz del proyecto:

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

Todo en uno:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Para salir del monitor: `Ctrl + ]`.

Limpiar build si algo se atasca:

```bash
idf.py fullclean
```

## Control desde el navegador

El ESP32 se conecta como STA a la red **NASA-Wi-Fi** (configurado en [main/wifi_module.c](main/wifi_module.c)) y recibe IP por DHCP.

1. Flashea y abre el monitor.
2. Busca en el log una línea como:

   ```
   I (xxxx) WIFI: Got IP: 192.168.x.x
   ```

3. Abre esa IP en el navegador:

   ```
   http://192.168.x.x/
   ```

   Sirve [main/index.html](main/index.html) y abre el WebSocket en `ws://192.168.x.x/ws` para telemetría y comandos.

## Si el puerto serial está ocupado

En Linux, cuando `idf.py flash` falla porque otro proceso tiene tomado `/dev/ttyUSB0` (típicamente un `idf.py monitor` previo que no cerró bien):

```bash
sudo fuser -k /dev/ttyUSB0
```

Eso mata al proceso que lo retiene y libera el puerto. Después se puede volver a flashear normalmente.

Si el puerto ni siquiera aparece, revisa permisos:

```bash
ls -l /dev/ttyUSB0
sudo usermod -aG dialout $USER   # requiere cerrar sesión para aplicar
```
