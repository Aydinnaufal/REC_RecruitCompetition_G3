#pragma once
#include "Arduino.h"
typedef enum { ESP_MAC_WIFI_STA = 0, ESP_MAC_BT = 2 } esp_mac_type_t;
inline int esp_read_mac(uint8_t* mac, esp_mac_type_t) { const uint8_t m[6] = {0xA0, 0xB1, 0xC2, 0xD3, 0xE4, 0xF5}; memcpy(mac, m, 6); return 0; }
