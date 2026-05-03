#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wifi_manager.h"
#include <string.h>

static const char *TAG = "wifi_manager";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_STARTED_BIT   BIT2

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t wifi_event_group;
static esp_netif_ip_info_t ip_info = {0};

static uint8_t retry_count = 0;
static uint8_t max_retries = 3;
static bool scanning = false;
/* Set by wifi_disconnect() so STA_DISCONNECTED does not auto-retry esp_wifi_connect() */
static bool s_intentional_disconnect = false;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(wifi_event_group, WIFI_STARTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP) {
        xEventGroupClearBits(wifi_event_group, WIFI_STARTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        if (s_intentional_disconnect) {
            s_intentional_disconnect = false;
            retry_count = 0;
            return;
        }
        if (retry_count < max_retries) {
            esp_wifi_connect();
            retry_count++;
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        memcpy(&ip_info, &event->ip_info, sizeof(ip_info));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static inline void wifi_ap_print_info(wifi_ap_record_t* record) {
    // Make a string representation of BSSID.
    char* bssid_str = malloc(3 * 6);
    if (!bssid_str)
        return;
    snprintf(
        bssid_str,
        3 * 6,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        record->bssid[0],
        record->bssid[1],
        record->bssid[2],
        record->bssid[3],
        record->bssid[4],
        record->bssid[5]
    );

    // Make a string representation of 11b/g/n modes.
    char* phy_str = malloc(9);
    if (!phy_str) {
        free(bssid_str);
        return;
    }
    *phy_str = 0;
    if (record->phy_11b | record->phy_11g | record->phy_11n) {
        strcpy(phy_str, " 1");
    }
    if (record->phy_11b) {
        strcat(phy_str, "/b");
    }
    if (record->phy_11g) {
        strcat(phy_str, "/g");
    }
    if (record->phy_11n) {
        strcat(phy_str, "/n");
    }
    phy_str[2] = '1';

    ESP_LOGI(TAG, "AP %s %s rssi=%hhd%s", bssid_str, record->ssid, record->rssi, phy_str);
    free(bssid_str);
    free(phy_str);
}


void wifi_connect(const char* ssid, const char* password, wifi_auth_mode_t auth_mode, uint8_t retry_max) {
    retry_count = 0;
    max_retries = retry_max;

    wifi_config_t wifi_config = {0};

    memcpy((char*)wifi_config.sta.ssid, ssid, strnlen(ssid, 32));
    memcpy((char*)wifi_config.sta.password, password, strnlen(password, 64));
    wifi_config.sta.threshold.authmode = auth_mode;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
}

void wifi_connect_to_stored() {

    wifi_auth_mode_t authmode = 0;
    char* ssid = NULL;
    char* password = NULL;
    size_t len;

    // Open NVS
    nvs_handle_t handle;
    nvs_open("system", NVS_READWRITE, &handle);

    esp_err_t ret;

    // Get SSID
    ret = nvs_get_str(handle, "wifi.ssid", NULL, &len);
    if(ret)
        goto errcheck;

    ssid = malloc(len);
    ret  = nvs_get_str(handle, "wifi.ssid", ssid, &len);
    if(ret)
        goto errcheck;

    // Get authmode
    ret = nvs_get_u8(handle, "wifi.authmode", (uint8_t*)&authmode);
    if (ret)
        goto errcheck;

    // Get password
    ret = nvs_get_str(handle, "wifi.password", NULL, &len);
    if (ret)
        goto errcheck;
   
    password = malloc(len);
    ret  = nvs_get_str(handle, "wifi.password", password, &len);
    if (ret)
        goto errcheck;

    nvs_close(handle);

    errcheck:
    if(ret == ESP_ERR_NVS_NOT_FOUND || ret == ESP_ERR_NVS_NOT_INITIALIZED) {
        ESP_LOGE(TAG, "Failed to read WiFi configuration from NVS");
    } else if (ret) {
        // Other errors.
        ESP_LOGE(TAG, "Error connecting to WiFi: %s", esp_err_to_name(ret));
    }

    // Free memory.
    if(ssid)
        free(ssid);
    if(password)
        free(password);

}

void wifi_disconnect() {
    s_intentional_disconnect = true;
    esp_wifi_disconnect();
}

bool wifi_is_scanning(void) {
    return scanning;
}


void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_init_event_handlers();
}

void wifi_init_event_handlers() {
    // Create an event group for WiFi things.
    wifi_event_group = xEventGroupCreate();

    // Register event handlers for WiFi.
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id)
    );
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip)
    );
}

size_t wifi_scan_aps(wifi_ap_record_t** aps_list) {
    scanning = true;
    wifi_ap_record_t* aps = NULL;

    /* Force standard 11b/g/n so the scan can see normal APs (LR mode can't) */
    esp_wifi_set_protocol(WIFI_IF_STA,
                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    vTaskDelay(pdMS_TO_TICKS(100));

    wifi_scan_config_t cfg = {
        .ssid      = NULL,
        .bssid     = NULL,
        .channel   = 0,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {.active = {.min = 120, .max = 300}},
    };

    ESP_LOGI(TAG, "Scanning for WiFi access points");
    esp_err_t ret = esp_wifi_scan_start(&cfg, true);
    if (ret) {
        ESP_LOGE(TAG, "Error in WiFi scan: %s", esp_err_to_name(ret));
        scanning = false;
        return 0;
    }

    uint16_t num_ap = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&num_ap));

    if (num_ap == 0) {
        ESP_LOGI(TAG, "Scan complete — no APs found");
        scanning = false;
        if (aps_list) *aps_list = NULL;
        return 0;
    }

    aps = malloc(sizeof(wifi_ap_record_t) * num_ap);
    if (!aps) {
        ESP_LOGE(TAG, "Out of memory (%zd bytes)", sizeof(wifi_ap_record_t) * num_ap);
        num_ap = 0;
        esp_wifi_scan_get_ap_records(&num_ap, NULL);
        return 0;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_ap, aps));
    for (int i = 0; i < num_ap; i++) {
        wifi_ap_print_info(&aps[i]);
    }

    if (aps_list)
        *aps_list = aps;
    else
        free(aps);

    scanning = false;
    return num_ap;
}
