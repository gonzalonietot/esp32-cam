/*
 * Prueba de camara ESP32-CAM (AI Thinker + OV2640)
 *
 * Que hace:
 *   1. Autodiagnostico por Monitor Serie: motivo del ultimo reinicio (detecta
 *      brownout), PSRAM, sensor detectado y 3 capturas de prueba con tiempos.
 *   2. Se conecta al WiFi (include/secrets.h). Si no puede en 20 s, crea su
 *      propia red "ESP32-CAM-PRUEBA" -> abrir http://192.168.4.1
 *   3. Servidor web:
 *        http://<IP>/            Pagina de prueba (foto, stream, flash, ajustes)
 *        http://<IP>/capture     Foto JPEG     (igual que el firmware de AnimalSense)
 *        http://<IP>/status      Diagnostico JSON (amplia el de AnimalSense)
 *        http://<IP>/control?var=X&val=Y   framesize|quality|brightness|contrast|vflip|hmirror|led
 *        http://<IP>:81/stream   Video MJPEG (el que menciona CameraModal.vue)
 *
 * Conexiones / hardware:
 *   - Alimentar con 5V estables (>= 1A). Con 3.3V o un USB flojo aparecen
 *     reinicios por "brownout" en cuanto arranca el WiFi.
 *   - Para cargar el programa: base ESP32-CAM-MB, o adaptador USB-TTL con
 *     GPIO0 a GND durante el upload (quitar el puente y pulsar RST despues).
 *   - LED rojo de la placa (GPIO 33): parpadea si la camara no inicia.
 *   - LED flash (GPIO 4): se controla desde la pagina.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "secrets.h"
#include <atomic>

// ---------- Pines AI Thinker ESP32-CAM ----------
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

#define LED_FLASH_PIN    4   // LED blanco frontal (activo en alto)
#define LED_ROJO_PIN    33   // LED rojo trasero (activo en BAJO)

// ---------- Estado ----------
static httpd_handle_t webHttpd = nullptr;
static httpd_handle_t streamHttpd = nullptr;
static bool modoAP = false;
static bool ledEncendido = false;
static uint32_t totalCapturas = 0;
static uint32_t totalErrores = 0;
static volatile float streamFps = 0;
static std::atomic<uint8_t> clientesStream{0};

// ============================================================================
// Utilidades
// ============================================================================
static const char* motivoReinicio(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "encendido";
    case ESP_RST_SW:       return "reinicio por software";
    case ESP_RST_PANIC:    return "PANIC (error de programa)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (alimentacion insuficiente)";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    case ESP_RST_EXT:      return "pin RST";
    default:               return "desconocido";
  }
}

static const char* nombreSensor(uint16_t pid) {
  switch (pid) {
    case OV2640_PID: return "OV2640";
    case OV3660_PID: return "OV3660";
    case OV5640_PID: return "OV5640";
    case OV7725_PID: return "OV7725";
    default:         return "desconocido";
  }
}

static void setLed(bool on) {
  ledEncendido = on;
  digitalWrite(LED_FLASH_PIN, on ? HIGH : LOW);
}

static String ipActual() {
  return modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

// ============================================================================
// Camara
// ============================================================================
static esp_err_t iniciarCamara() {
  // Inicializar en {} es importante: camera_config_t tiene campos (grab_mode,
  // fb_location, sccb_i2c_port...) que si quedan con basura dan fallos raros.
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    // Se inicia en UXGA para reservar buffers grandes y poder cambiar la
    // resolucion despues desde la pagina sin quedarse sin memoria.
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;  // siempre la foto mas reciente
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return err;

  // Resolucion de trabajo: VGA (640x480) igual que AnimalSense
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, psramFound() ? FRAMESIZE_VGA : FRAMESIZE_CIF);
  return ESP_OK;
}

static void pruebaDeCaptura() {
  Serial.println("Prueba de captura (3 fotos):");
  for (int i = 1; i <= 3; i++) {
    uint32_t t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    uint32_t dt = millis() - t0;
    if (!fb) {
      Serial.printf("  #%d  ERROR: no se obtuvo frame\n", i);
      totalErrores++;
      continue;
    }
    Serial.printf("  #%d  OK  %ux%u  %u KB  %u ms\n", i, fb->width, fb->height,
                  (unsigned)(fb->len / 1024), (unsigned)dt);
    esp_camera_fb_return(fb);
  }
}

static void errorCamaraFatal(esp_err_t err) {
  Serial.printf("ERROR iniciando camara: 0x%x (%s)\n", err, esp_err_to_name(err));
  if (err == ESP_ERR_NOT_FOUND || err == 0x20001 || err == 0x20002) {
    Serial.println("  -> No se detecto el sensor: revisar la cinta (flex) del modulo,");
    Serial.println("     que este bien insertada y con la traba cerrada.");
  } else if (err == ESP_ERR_NO_MEM) {
    Serial.println("  -> Sin memoria: la placa no tiene PSRAM o no esta habilitada.");
  }
  Serial.println("  Reiniciando en 10 s...");
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    digitalWrite(LED_ROJO_PIN, !digitalRead(LED_ROJO_PIN));
    delay(150);
  }
  ESP.restart();
}

// ============================================================================
// WiFi
// ============================================================================
static void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // menos latencia en el stream
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Conectando a WiFi '%s'", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    modoAP = false;
    Serial.printf("WiFi OK  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return;
  }

  Serial.println("No se pudo conectar. Creando red propia para probar la camara...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  modoAP = true;
  Serial.printf("Red: '%s'  clave: '%s'  ->  http://%s\n", AP_SSID, AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
}

// ============================================================================
// Pagina de prueba
// ============================================================================
static const char INDEX_HTML[] = R"rawliteral(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM prueba</title>
<style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0 auto;padding:16px;max-width:900px}
h1{font-size:1.2rem;margin:0 0 8px}
.row{display:flex;flex-wrap:wrap;gap:8px;margin:8px 0;align-items:center}
button,select{background:#2a2a2a;color:#eee;border:1px solid #444;border-radius:6px;padding:8px 12px;font-size:14px;cursor:pointer}
button.on{background:#2563eb;border-color:#2563eb}
label{font-size:14px;display:flex;gap:6px;align-items:center}
img{display:block;width:100%;background:#000;border-radius:8px;min-height:240px;object-fit:contain}
#info{color:#9ca3af;font-size:13px;min-height:1em}
pre{background:#1b1b1b;padding:10px;border-radius:6px;font-size:12px;overflow:auto}
</style></head><body>
<h1>ESP32-CAM &middot; prueba</h1>
<div class="row">
  <button id="snap">Foto</button>
  <button id="stream">Iniciar stream</button>
  <button id="led">Flash</button>
  <select id="framesize" title="Resolucion">
    <option value="5">QVGA 320x240</option><option value="6">CIF 400x296</option>
    <option value="8">VGA 640x480</option><option value="9">SVGA 800x600</option>
    <option value="10">XGA 1024x768</option><option value="11">HD 1280x720</option>
    <option value="12">SXGA 1280x1024</option><option value="13">UXGA 1600x1200</option>
  </select>
</div>
<div class="row">
  <label>Calidad JPEG <input id="quality" type="range" min="10" max="40" value="12"></label>
  <label>Brillo <input id="brightness" type="range" min="-2" max="2" value="0"></label>
  <label><input type="checkbox" id="vflip"> Voltear</label>
  <label><input type="checkbox" id="hmirror"> Espejo</label>
</div>
<img id="img" alt="">
<p id="info"></p>
<pre id="st">Cargando estado...</pre>
<script>
const $=id=>document.getElementById(id);
let streaming=false;
function stopStream(){
  if(!streaming)return;
  $('img').src='';streaming=false;
  $('stream').textContent='Iniciar stream';$('stream').classList.remove('on');
}
$('snap').onclick=()=>{
  stopStream();
  const t0=performance.now();
  $('img').onload=()=>{$('info').textContent='Foto '+$('img').naturalWidth+'x'+$('img').naturalHeight+' en '+Math.round(performance.now()-t0)+' ms';};
  $('img').onerror=()=>{$('info').textContent='Error al obtener la foto';};
  $('img').src='/capture?t='+Date.now();
};
$('stream').onclick=()=>{
  if(streaming){stopStream();return;}
  const url='http://'+location.hostname+':81/stream';
  $('img').onload=null;$('img').onerror=()=>{$('info').textContent='Error en el stream';stopStream();};
  $('img').src=url;streaming=true;
  $('stream').textContent='Detener stream';$('stream').classList.add('on');
  $('info').textContent='Stream MJPEG: '+url;
};
function ctrl(v,x){return fetch('/control?var='+v+'&val='+x).then(r=>{if(!r.ok)$('info').textContent='No se pudo cambiar '+v;});}
$('led').onclick=()=>{const on=!$('led').classList.contains('on');ctrl('led',on?1:0).then(()=>$('led').classList.toggle('on',on));};
['framesize','quality','brightness'].forEach(id=>$(id).onchange=e=>ctrl(id,e.target.value));
['vflip','hmirror'].forEach(id=>$(id).onchange=e=>ctrl(id,e.target.checked?1:0));
let first=true;
async function status(){
  try{
    const s=await (await fetch('/status')).json();
    $('st').textContent=JSON.stringify(s,null,2);
    if(first){first=false;
      $('framesize').value=s.framesize;$('quality').value=s.quality;$('brightness').value=s.brightness;
      $('vflip').checked=!!s.vflip;$('hmirror').checked=!!s.hmirror;$('led').classList.toggle('on',!!s.led);
    }
  }catch(e){$('st').textContent='Sin respuesta de /status';}
}
status();setInterval(status,3000);
</script></body></html>)rawliteral";

// ============================================================================
// Handlers HTTP
// ============================================================================
static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    totalErrores++;
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error capturando frame de la camara");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  totalCapturas++;
  return res;
}

static esp_err_t statusHandler(httpd_req_t* req) {
  sensor_t* s = esp_camera_sensor_get();
  String ip = ipActual();
  char json[900];
  snprintf(json, sizeof(json),
    "{\"device\":\"esp32-cam-prueba\",\"ip\":\"%s\",\"modo\":\"%s\","
    "\"captureUrl\":\"http://%s/capture\",\"streamUrl\":\"http://%s:81/stream\","
    "\"rssi\":%d,\"uptimeS\":%lu,\"heapLibre\":%u,\"psram\":%s,\"psramLibre\":%u,"
    "\"sensor\":\"%s\",\"framesize\":%d,\"quality\":%d,\"brightness\":%d,"
    "\"vflip\":%d,\"hmirror\":%d,\"led\":%d,"
    "\"capturas\":%lu,\"errores\":%lu,\"clientesStream\":%u,\"streamFps\":%.1f,"
    "\"ultimoReinicio\":\"%s\"}",
    ip.c_str(), modoAP ? "AP" : "STA", ip.c_str(), ip.c_str(),
    modoAP ? 0 : WiFi.RSSI(), (unsigned long)(millis() / 1000),
    (unsigned)ESP.getFreeHeap(), psramFound() ? "true" : "false", (unsigned)ESP.getFreePsram(),
    nombreSensor(s->id.PID), (int)s->status.framesize, s->status.quality, s->status.brightness,
    s->status.vflip, s->status.hmirror, ledEncendido ? 1 : 0,
    (unsigned long)totalCapturas, (unsigned long)totalErrores, (unsigned)clientesStream.load(),
    (double)streamFps, motivoReinicio(esp_reset_reason()));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t controlHandler(httpd_req_t* req) {
  char query[64], var[16], val[8];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Uso: /control?var=...&val=...");
    return ESP_FAIL;
  }
  int v = atoi(val);
  sensor_t* s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(var, "framesize")) {
    if (v < FRAMESIZE_QVGA || v > FRAMESIZE_UXGA) res = -1;
    else if (!psramFound() && v > FRAMESIZE_SVGA) res = -1;  // sin PSRAM no entra
    else res = s->set_framesize(s, (framesize_t)v);
  }
  else if (!strcmp(var, "quality"))    res = s->set_quality(s, constrain(v, 10, 63));
  else if (!strcmp(var, "brightness")) res = s->set_brightness(s, constrain(v, -2, 2));
  else if (!strcmp(var, "contrast"))   res = s->set_contrast(s, constrain(v, -2, 2));
  else if (!strcmp(var, "vflip"))      res = s->set_vflip(s, v ? 1 : 0);
  else if (!strcmp(var, "hmirror"))    res = s->set_hmirror(s, v ? 1 : 0);
  else if (!strcmp(var, "led"))        setLed(v != 0);
  else res = -1;

  if (res != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parametro o valor no valido");
    return ESP_FAIL;
  }
  Serial.printf("Control: %s = %d\n", var, v);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  clientesStream++;
  Serial.println("Stream: cliente conectado");

  char part[64];
  uint32_t frames = 0, tVentana = millis();
  esp_err_t res = ESP_OK;
  while (res == ESP_OK) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      totalErrores++;
      res = ESP_FAIL;
      break;
    }
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);

    frames++;
    uint32_t dt = millis() - tVentana;
    if (dt >= 2000) {
      streamFps = frames * 1000.0f / dt;
      frames = 0;
      tVentana = millis();
    }
  }

  if (--clientesStream == 0) streamFps = 0;
  Serial.println("Stream: cliente desconectado");
  return res;
}

static void iniciarServidores() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  cfg.max_uri_handlers = 8;

  httpd_uri_t uriIndex   = { .uri = "/",        .method = HTTP_GET, .handler = indexHandler,   .user_ctx = nullptr };
  httpd_uri_t uriCapture = { .uri = "/capture", .method = HTTP_GET, .handler = captureHandler, .user_ctx = nullptr };
  httpd_uri_t uriStatus  = { .uri = "/status",  .method = HTTP_GET, .handler = statusHandler,  .user_ctx = nullptr };
  httpd_uri_t uriControl = { .uri = "/control", .method = HTTP_GET, .handler = controlHandler, .user_ctx = nullptr };
  httpd_uri_t uriStream  = { .uri = "/stream",  .method = HTTP_GET, .handler = streamHandler,  .user_ctx = nullptr };

  if (httpd_start(&webHttpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(webHttpd, &uriIndex);
    httpd_register_uri_handler(webHttpd, &uriCapture);
    httpd_register_uri_handler(webHttpd, &uriStatus);
    httpd_register_uri_handler(webHttpd, &uriControl);
  } else {
    Serial.println("ERROR: no arranco el servidor del puerto 80");
  }

  // El stream va en otro servidor/puerto para que la pagina y /capture
  // sigan respondiendo mientras hay un video abierto.
  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  if (httpd_start(&streamHttpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(streamHttpd, &uriStream);
  } else {
    Serial.println("ERROR: no arranco el servidor del puerto 81");
  }
}

// ============================================================================
// Setup / loop
// ============================================================================
void setup() {
  pinMode(LED_FLASH_PIN, OUTPUT);
  pinMode(LED_ROJO_PIN, OUTPUT);
  setLed(false);
  digitalWrite(LED_ROJO_PIN, HIGH);  // apagado (activo en bajo)

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-CAM | prueba de camara ===");
  Serial.printf("Ultimo reinicio: %s\n", motivoReinicio(esp_reset_reason()));
  Serial.printf("Chip: %s rev %d, %d MHz | Heap libre: %u KB\n", ESP.getChipModel(),
                ESP.getChipRevision(), (int)ESP.getCpuFreqMHz(), (unsigned)(ESP.getFreeHeap() / 1024));
  if (psramFound()) {
    Serial.printf("PSRAM: %u KB\n", (unsigned)(ESP.getPsramSize() / 1024));
  } else {
    Serial.println("PSRAM: NO detectada (resolucion maxima limitada a SVGA)");
  }

  esp_err_t err = iniciarCamara();
  if (err != ESP_OK) errorCamaraFatal(err);

  sensor_t* s = esp_camera_sensor_get();
  Serial.printf("Camara OK: sensor %s (PID 0x%x)\n", nombreSensor(s->id.PID), s->id.PID);
  pruebaDeCaptura();

  iniciarWiFi();
  iniciarServidores();

  String ip = ipActual();
  Serial.println("----------------------------------------------");
  Serial.printf("Pagina de prueba:  http://%s/\n", ip.c_str());
  Serial.printf("Foto (AnimalSense): http://%s/capture\n", ip.c_str());
  Serial.printf("Stream MJPEG:      http://%s:81/stream\n", ip.c_str());
  Serial.printf("Diagnostico:       http://%s/status\n", ip.c_str());
  Serial.println("----------------------------------------------");
}

void loop() {
  static uint32_t ultimoLog = 0, ultimoIntento = 0;

  // Reconexion WiFi (el firmware original no la tiene)
  if (!modoAP && WiFi.status() != WL_CONNECTED && millis() - ultimoIntento > 10000) {
    ultimoIntento = millis();
    Serial.println("WiFi caido, reintentando...");
    WiFi.reconnect();
  }

  if (millis() - ultimoLog > 10000) {
    ultimoLog = millis();
    Serial.printf("[%lus] %s  RSSI:%d  heap:%uKB  capturas:%lu  errores:%lu  stream:%u (%.1f fps)\n",
                  (unsigned long)(millis() / 1000), ipActual().c_str(),
                  modoAP ? 0 : WiFi.RSSI(), (unsigned)(ESP.getFreeHeap() / 1024),
                  (unsigned long)totalCapturas, (unsigned long)totalErrores,
                  (unsigned)clientesStream.load(),(double)streamFps);
  }
  delay(50);
}
