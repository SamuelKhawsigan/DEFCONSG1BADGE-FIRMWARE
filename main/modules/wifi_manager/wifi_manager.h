#pragma once

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


void wifi_connect(const char* ssid, const char* password, wifi_auth_mode_t auth_mode, uint8_t retry_max);
void wifi_connect_to_stored();
void wifi_disconnect();
void wifi_init();
void wifi_init_event_handlers();
size_t wifi_scan_aps(wifi_ap_record_t** aps_list);
bool wifi_is_scanning(void);
