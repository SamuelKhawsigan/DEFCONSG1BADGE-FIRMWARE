#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "frotz.h"
#include "esp_spiffs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "ta_parser.h"
#include "lasertag.h"
#include "tvbgone_badge.h"
#include "creator_detector.h"
#include "libneon_led_controller.h"
#include "wifi_manager.h"
#include "lcd_display.h"

/* Set to 1 for one-time WiFi scan + open-AP connect test, then set back to 0 */
// Feel free to remove this, just sharing to help in quick test.
#define WIFI_CONNECT_TEST      0
#define WIFI_CONNECT_TEST_SSID "NameOfOpenNetworkToTestWith"

#if WIFI_CONNECT_TEST
#include "esp_netif.h"
#include "esp_wifi.h"
static const char *WIFI_TEST_TAG = "wifi_test";
#endif

static const char *TAG = "app_main";

/* ── Button definitions ──────────────────────────────────────────────────── */

typedef void (*button_action_fn)(void);

typedef struct {
    const char      *name;
    int              gpio_num;
    button_action_fn short_press_action;
    button_action_fn long_press_action;
} button_def_t;

static const button_def_t button_defs[] = {
    { "A", 23, led_controller_decrease_brightness, NULL                         },
    { "B", 15, led_controller_increase_brightness, NULL                         },
    { "C", 10, led_controller_next_animation,      NULL                         },
    { "D",  9, lasertag_fire_button_pressed,        lasertag_fire_button_pressed },
};

#define NUM_BUTTONS (sizeof(button_defs) / sizeof(button_defs[0]))

/* ── Generic button callbacks ────────────────────────────────────────────── */

static inline bool is_button_a(const button_def_t *def)
{
    return def == &button_defs[0];
}

static void on_short_press(void *arg, void *usr_data)
{
    const button_def_t *def = (const button_def_t *)usr_data;
    ESP_LOGI(TAG, "Button %s (GPIO %d): SHORT press", def->name, def->gpio_num);

    if (tvbgone_badge_is_running()) {
        tvbgone_badge_stop();
    }

    if (def->short_press_action) def->short_press_action();
}

static void on_long_press(void *arg, void *usr_data)
{
    const button_def_t *def = (const button_def_t *)usr_data;
    ESP_LOGI(TAG, "Button %s (GPIO %d): LONG press", def->name, def->gpio_num);

    bool was_running = tvbgone_badge_is_running();

    if (was_running) {
        tvbgone_badge_stop();
    }

    if (is_button_a(def)) {
        if (!was_running) {
            tvbgone_badge_start();
        }
        return;
    }

    if (def->long_press_action) def->long_press_action();
}

/* ── Button initialization ───────────────────────────────────────────────── */

static void init_buttons(void)
{
    const button_config_t btn_cfg = {
        .short_press_time = 50,
    };

    for (size_t i = 0; i < NUM_BUTTONS; i++) {
        const button_def_t *def = &button_defs[i];
        const button_gpio_config_t gpio_cfg = {
            .gpio_num     = def->gpio_num,
            .active_level = 0,
        };

        button_handle_t btn = NULL;
        ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn));
        ESP_ERROR_CHECK(iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL,
                                               on_short_press, (void *)def));
        ESP_ERROR_CHECK(iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL,
                                               on_long_press, (void *)def));
        ESP_LOGI(TAG, "Button %s registered on GPIO %d", def->name, def->gpio_num);
    }
}

/* ── Frotz / text-adventure task ─────────────────────────────────────────── */

static void ta_parser_task(void *arg)
{
    while (1) ta_parser_main();
}

// FEEL FREE TO REMOVE THIS, just sharing to help in quick test.
#if WIFI_CONNECT_TEST
static void wifi_connect_test_task(void *arg)
{

    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(10000));

    wifi_ap_record_t *aps = NULL;
    size_t n = wifi_scan_aps(&aps);
    ESP_LOGI(WIFI_TEST_TAG, "Scan: %u APs (see lines below, channel=primary):", (unsigned)n);
    for (size_t i = 0; i < n; i++) {
        ESP_LOGI(WIFI_TEST_TAG, "  [%u] ch=%u rssi=%d auth=%d ssid=%s", (unsigned)i,
                 (unsigned)aps[i].primary, (int)aps[i].rssi, (int)aps[i].authmode,
                 (const char *)aps[i].ssid);
    }
    if (aps) {
        free(aps);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    const char *ssid = WIFI_CONNECT_TEST_SSID;
    ESP_LOGI(WIFI_TEST_TAG, "Connecting to open network \"%s\" ...", ssid);
    wifi_connect(ssid, "", WIFI_AUTH_OPEN, 3);

    bool got_ip = false;
    for (int step = 0; step < 60; step++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
            continue;
        }
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!netif) {
            continue;
        }
        esp_netif_ip_info_t ipi;
        if (esp_netif_get_ip_info(netif, &ipi) == ESP_OK && ipi.ip.addr != 0) {
            ESP_LOGI(WIFI_TEST_TAG,
                     "OK: associated ch=%u rssi=%d IP=" IPSTR,
                     (unsigned)ap.primary, (int)ap.rssi, IP2STR(&ipi.ip));
            got_ip = true;
            break;
        }
    }
    if (!got_ip) {
        ESP_LOGW(WIFI_TEST_TAG, "No IP in 30s (SSID in range? still open?)");
    }

    vTaskDelay(pdMS_TO_TICKS(60000));

    ESP_LOGI(WIFI_TEST_TAG, "Disconnecting ...");
    wifi_disconnect();
    ESP_LOGI(WIFI_TEST_TAG, "Test finished.");
    vTaskDelete(NULL);
}
#endif

/* ═════════════════════════════════════════════════════════════════════════════
 *  Application entry point
 * ═════════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    ESP_LOGI(TAG, "══════════════════════════════════════");
    ESP_LOGI(TAG, " Badge Firmware — ESP-IDF");
    ESP_LOGI(TAG, "══════════════════════════════════════");

    /* ── USB Serial JTAG (for Frotz console I/O) ─────────────────────────── */
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = 128,
        .tx_buffer_size = 128,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    usb_serial_jtag_vfs_use_nonblocking();
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* ── NVS (shared across all modules) ──────────────────────────────────── */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* ── SPIFFS (game data for Frotz) ─────────────────────────────────────── */
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false,
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    /* ── Ensure player identity exists in NVS (needed by LED startup wipe for first boot) ── */
    lasertag_init_player_config();

    /* ── Start LED animation controller (must precede lasertag_start) ─────── */
    led_controller_start();

    /* ── WiFi stack init (shared by wifi_manager and ESP-NOW in lasertag) ── */
    wifi_init();

    /* ── Start lasertag module (background hit detection + auto sync) ─────── */
    lasertag_start();

    /* ── TV-B-Gone (borrows the lasertag IR TX channel) ───────────────────── */
    tvbgone_badge_init();

    /* ── Start creator-detector BLE scanner ───────────────────────────────── */
    creator_detector_start();

    /*
     * Buttons after WiFi + BLE: RF calibration / coexistence can couple noise onto
     * GPIO lines. The IoT Button driver starts with debounced level = "released"
     * without sampling the pin; an early false "pressed" reading wedges the state
     * machine until the line is stable (often a few seconds).
     */
    init_buttons();

    /* ── Start Frotz / text-adventure console ─────────────────────────────── */
    xTaskCreate(ta_parser_task, "ta_parser", 8192, NULL, 5, NULL);

    /* ── Start LCD display ────────────────────────────────────────────────── */
    lcd_display_init();
    lcd_display_clear();
    lcd_display_show_jpg("/spiffs/rengoku_ready.jpg");
    // lcd_display_test_pattern();

    /* ── Match LED animation to the displayed image (3-color palette) ───────── */
    {
        uint8_t r1=0,g1=0,b1=0, r2=0,g2=0,b2=0, r3=0,g3=0,b3=0;
        lcd_display_get_palette(&r1,&g1,&b1, &r2,&g2,&b2, &r3,&g3,&b3);
        led_controller_image_palette_start(r1,g1,b1, r2,g2,b2, r3,g3,b3);
    }

#if WIFI_CONNECT_TEST
    xTaskCreate(wifi_connect_test_task, "wifi_conn_test", 4096, NULL, 2, NULL);
#endif
}
