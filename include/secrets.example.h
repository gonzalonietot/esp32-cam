// Copia este archivo como "secrets.h" (en esta misma carpeta) y completa tus datos.
// secrets.h esta en .gitignore para no subir la clave del WiFi al repositorio.
#pragma once

#define WIFI_SSID     "TU_WIFI_SSID"
#define WIFI_PASSWORD "TU_WIFI_PASSWORD"

// Si no logra conectarse al WiFi en WIFI_TIMEOUT_MS, la camara crea su propia
// red para poder probarla igual: conectate a ella y abre http://192.168.4.1
#define AP_SSID       "ESP32-CAM-PRUEBA"
#define AP_PASSWORD   "camara1234"   // minimo 8 caracteres
#define WIFI_TIMEOUT_MS 20000
