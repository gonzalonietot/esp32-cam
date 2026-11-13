# ESP32-CAM · proyecto de prueba

Proyecto PlatformIO (igual que `sensores-esp32`) para probar una **ESP32-CAM AI Thinker** con sensor OV2640 antes de usarla en AnimalSense (`animal-monitoring`).

## Qué hace

- **Autodiagnóstico en el Monitor Serie**: motivo del último reinicio (detecta *brownout*), PSRAM, sensor detectado y 3 fotos de prueba con tamaño y tiempo.
- **Página de prueba** en `http://<IP>/`: foto, video en vivo, flash, resolución, calidad, brillo, voltear/espejo y el estado en vivo.
- **Endpoints compatibles con AnimalSense**:

| URL | Qué devuelve |
|---|---|
| `http://<IP>/capture` | Foto JPEG (el mismo endpoint que usa el backend) |
| `http://<IP>:81/stream` | Video MJPEG (el que menciona `CameraModal.vue`) |
| `http://<IP>/status` | JSON con IP, RSSI, memoria, sensor, fps, errores, último reinicio |
| `http://<IP>/control?var=X&val=Y` | `framesize`, `quality`, `brightness`, `contrast`, `vflip`, `hmirror`, `led` |

- **Si el WiFi falla** (clave mal puesta, red lejos), en 20 s crea su propia red `ESP32-CAM-PRUEBA` (clave `camara1234`). Te conectás a ella y abrís `http://192.168.4.1`.

## Uso

1. Poné tu WiFi en `include/secrets.h` (no se sube a git; `secrets.example.h` es la plantilla).
2. Conectá la placa:
   - Con la base **ESP32-CAM-MB**: USB directo.
   - Con un **adaptador USB-TTL**: 5V→5V, GND→GND, TX→U0R, RX→U0T, y **GPIO0 a GND** para cargar. Después quitá el puente y pulsá RST.
3. Cargá y abrí el monitor:
   ```bash
   pio run -t upload -t monitor
   ```
4. Abrí la URL que aparece en el monitor (`Pagina de prueba: http://...`).
5. **Varias cámaras a la vez:** abrí `camaras.html` (doble clic) desde una PC en la misma red. Busca las cámaras en la red (`/status` con `captureUrl`) o las agregás por IP, y desde ahí tenés foto, video, flash, ajustes y estado de cada una. Recuerda la lista entre sesiones.
6. Para AnimalSense: pegá `http://<IP>/capture` (o `http://<IP>:81/stream`) en la cámara desde la vista **Dispositivos**.

### Problemas comunes

| Síntoma | Causa probable |
|---|---|
| `Ultimo reinicio: BROWNOUT` o se reinicia al conectar WiFi | Fuente débil. Usar 5V ≥ 1A y un cable USB corto/bueno. |
| `ERROR iniciando camara: 0x105` / `0x20001` | La cinta de la cámara está floja o la traba abierta. |
| `PSRAM: NO detectada` | Placa sin PSRAM o clon. Funciona pero con resolución máxima SVGA. |
| No carga el programa (`Failed to connect`) | Falta GPIO0 a GND, o pulsar RST justo al empezar la carga. |
| El monitor no muestra nada con la base MB | Ya está resuelto con `monitor_rts = 0` / `monitor_dtr = 0`; pulsá RST. |

---

## Análisis del firmware actual del proyecto

Archivo: `animal-monitoring/firmware/esp32-cam-galpon/esp32-cam-galpon.ino`

**Qué hace hoy:** inicia la cámara (VGA, JPEG calidad 12, 2 buffers con PSRAM), se conecta al WiFi y expone `/capture` (foto JPEG) y `/status` (JSON con IP y URL) en el puerto 80 con la librería `WebServer`. El backend (`cameras.service.ts`) hace *pull* periódico de `/capture`. La estructura y los pines AI Thinker son correctos y es compatible con el backend.

### Problemas encontrados (de más a menos importante)

1. **`camera_config_t config;` no se inicializa.** Los campos que no se asignan (`grab_mode`, `fb_location`, `sccb_i2c_port`) quedan con basura de la pila. Puede funcionar "de casualidad" o fallar de forma intermitente. Arreglo: `camera_config_t config = {};` y fijar `fb_location` y `grab_mode` explícitamente.
2. **Fotos atrasadas en `/capture`.** Con `fb_count = 2` y el modo por defecto (`CAMERA_GRAB_WHEN_EMPTY`), el buffer guarda frames viejos: como el backend pide una foto cada X segundos, recibe una imagen de hace segundos, no la actual. Arreglo: `config.grab_mode = CAMERA_GRAB_LATEST;` (ya implícito si se arregla el punto 1 así).
3. **WiFi sin timeout ni reconexión.** Si la clave está mal se queda colgada en `while (WiFi.status() != WL_CONNECTED)` para siempre sin avisar; y si el router se reinicia, la cámara no se reconecta nunca (el `loop()` solo atiende HTTP). En un galpón esto significa una cámara muerta hasta que alguien la desenchufe.
4. **Si falla la cámara, queda "zombie".** El `return` en `setup()` deja la placa sin WiFi ni servidor y sin reintentar. Mejor: avisar (LED rojo) y reiniciar con `ESP.restart()` tras unos segundos.
5. **El stream `:81/stream` no existe.** `CameraModal.vue` ofrece `http://IP:81/stream` como opción, pero el firmware solo tiene `/capture`. Si alguien pega esa URL, no anda.
6. **Credenciales en el código.** `WIFI_SSID`/`WIFI_PASSWORD` dentro del `.ino`: es fácil subir la clave a git. Mejor en un `secrets.h` ignorado.
7. **`WiFi.setSleep(false)` ausente.** El ahorro de energía del WiFi agrega latencia a cada pedido (afecta el timeout de 4.5 s del backend con señal débil).
8. **`/status` muy pobre para diagnosticar.** No informa RSSI, memoria, uptime ni motivo de reinicio; justo lo que hace falta para saber por qué una cámara del galpón se cae (casi siempre alimentación o señal).
9. **Detalle menor:** `WebServer` (síncrono) con envío manual vía `server.client().write()` funciona, pero bloquea todo mientras manda la foto. `esp_http_server` (el que usa este proyecto de prueba) es el recomendado por Espressif para cámara.

Todos estos puntos están resueltos en `src/main.cpp` de este proyecto, que se puede usar como base para actualizar el firmware del galpón manteniendo los mismos endpoints.
