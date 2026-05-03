#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_crt_bundle.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "wifi_manager.h"


static const char* TAG = "OTA_updater";

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR: ESP_LOGD(TAG, "HTTP_EVENT_ERROR"); break;
        case HTTP_EVENT_ON_CONNECTED: ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED"); break;
        case HTTP_EVENT_HEADERS_SENT: ESP_LOGD(TAG, "HTTP_EVENT_HEADERS_SENT"); break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA: ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len); break;
        case HTTP_EVENT_ON_FINISH: ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH"); break;
        case HTTP_EVENT_DISCONNECTED: ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED"); break;
        case HTTP_EVENT_REDIRECT: ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT"); break;
    }
    return ESP_OK;
}

static esp_err_t validate_image_header(esp_app_desc_t* new_app_info) {
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_app_desc_t         running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(
            TAG,
            "Running firmware version: %s, available firmware version: %s",
            running_app_info.version,
            new_app_info->version
        );
        if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
            ESP_LOGW(TAG, "Already up-to-date!");
            return ESP_FAIL;
        }
    } 
    else {
        ESP_LOGW(TAG, "Unable to check current firmware version");
    }

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client) {
    esp_err_t err = ESP_OK;
    err = esp_http_client_set_header(http_client, "Badge-Type", "DEFCON SG 1");
    return err;
}

void ota_update() {
    char* ota_url = "https://bitowl.online/badges/dcsgonefirm2026.bin";

    ESP_LOGI(TAG, "Starting OTA update");

    esp_http_client_config_t config = {
        .url                 = ota_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler       = _http_event_handler,
        .keep_alive_enable   = true
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb =
            _http_client_init_cb,  // Register a callback to be invoked after esp_http_client is initialized
    };

    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        esp_https_ota_abort(https_ota_handle);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }
    
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) {
        esp_https_ota_abort(https_ota_handle);
        ESP_LOGI(TAG, "Already up-to-date!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return;
    }

    esp_err_t ota_finish_err = ESP_OK;
    int percent_shown = -1;
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        int len_total = esp_https_ota_get_image_size(https_ota_handle);
        int len_read  = esp_https_ota_get_image_len_read(https_ota_handle);
        int percent   = (len_read * 100) / len_total;

        if ((percent / 10) != (percent_shown / 10)) {
            ESP_LOGI(TAG, "Downloading %d / %d (%d%%)", len_read, len_total, percent);
            percent_shown = percent;
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Updating... %d%%", percent);
        }
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        ESP_LOGE(TAG, "Update failed: Complete data was not received.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    } 
    else {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } 
        else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            } 
            else {
                ESP_LOGE(TAG, "Update failed: Unknown reason");
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        }
    }

    esp_https_ota_abort(https_ota_handle);
    esp_restart();

    // We should never get here
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

