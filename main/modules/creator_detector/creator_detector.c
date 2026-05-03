/*
 * creator_detector.c
 *
 * BLE passive scanner that detects nearby creator devices (message forwarders).
 * Each forwarder broadcasts a non-connectable BLE advertisement containing a
 * manufacturer-specific payload with a CR-format tag string describing LED
 * behaviour.  The module maintains a per-MAC packet map, periodically
 * evaluates the set of valid (non-expired) packets, and drives the LED
 * controller accordingly.
 *
 * Two packet formats are accepted:
 *
 * 1) CR manufacturer-data (custom forwarders):
 *    CR{r},{g},{b}|{animation hex}|{brightness idx or empty}|{priority or empty}|{persistence decisecs}
 *
 * 2) iBeacon (e.g. MokoSmart M1 beacons):
 *    UUID  = 43520001-0000-0000-0000-000000000000
 *    Major = (R << 8) | G
 *    Minor = (B << 8) | (animation << 4 | brightness)
 *    Brightness nibble 0xF = no override.
 *    Priority defaults to 0, persistence to 30 deciseconds (3 s).
 *
 * Target: ESP32-C6 (badge firmware)
 */

#include "creator_detector.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "libneon_led_controller.h"

static const char *TAG = "creator_det";

#define VERBOSE_LOGGING             0

/* ── BLE tag prefix ──────────────────────────────────────────────────────── */
#define BLE_TAG_PREFIX      "CR"
#define BLE_TAG_PREFIX_LEN  (sizeof(BLE_TAG_PREFIX) - 1)

/* ── iBeacon alternative format ──────────────────────────────────────────── */
#define IBEACON_MFG_DATA_LEN     25
#define IBEACON_DEFAULT_PERSIST_DS 50   /* 5 seconds — refreshed each broadcast */
#define IBEACON_DEFAULT_PRIORITY   4

static const uint8_t CREATOR_IBEACON_UUID[16] = {
    0x43, 0x52, 0x00, 0x01,   /* "CR" + version 1 */
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};

/* ── Scan parameters ─────────────────────────────────────────────────────── */
#define SCAN_INTERVAL_MS    600
#define SCAN_WINDOW_MS      300
#define SCAN_INTERVAL_UNITS ((SCAN_INTERVAL_MS * 1000) / 625)
#define SCAN_WINDOW_UNITS   ((SCAN_WINDOW_MS  * 1000) / 625)

/* ── Packet map ──────────────────────────────────────────────────────────── */
#define MAX_TRACKED_DEVICES 16
#define MAC_ADDR_LEN        6
#define EVAL_INTERVAL_MS    200
#define EVAL_TASK_STACK     4096
#define EXPIRY_GRACE_MS     10

typedef struct {
    uint8_t  mac[MAC_ADDR_LEN];
    uint8_t  r, g, b;
    uint8_t  animation_type;     /* 0 = none, 1 = stagger breath, 2 = solid color */
    int      brightness_override;/* -1 = no override, else BRIGHTNESS_LEVELS index */
    int      priority;
    int      persistence_ds;     /* tenths of a second, 1-600 */
    int8_t   rssi;
    int64_t  received_time_us;
    bool     valid;
} creator_packet_t;

static creator_packet_t   s_packet_map[MAX_TRACKED_DEVICES];
static SemaphoreHandle_t  s_packet_mutex;

/* ═════════════════════════════════════════════════════════════════════════════
 *  CR payload parser
 *
 *  Format: CR{r},{g},{b}|{anim hex}|{brightness or empty}|{priority or empty}|{persistence ds}
 *  Example: CR255,0,128|1||5|300   (30 seconds)
 * ═════════════════════════════════════════════════════════════════════════════ */

static bool parse_cr_payload(const char *tag, int tag_len, creator_packet_t *out)
{
    char buf[128];
    int copy_len = tag_len < (int)sizeof(buf) - 1 ? tag_len : (int)sizeof(buf) - 1;
    memcpy(buf, tag, copy_len);
    buf[copy_len] = '\0';

    int r, g, b;
    int consumed = 0;
    if (sscanf(buf, "CR%d,%d,%d%n", &r, &g, &b, &consumed) < 3) {
        return false;
    }

    char *p = buf + consumed;
    if (*p != '|') return false;
    p++;

    /* animation type (single hex digit) */
    char *delim = strchr(p, '|');
    if (!delim) return false;
    long anim = strtol(p, NULL, 16);
    p = delim + 1;

    /* brightness override: empty field means no override */
    delim = strchr(p, '|');
    if (!delim) return false;
    int brightness = -1;
    if (p != delim) {
        brightness = (int)strtol(p, NULL, 10);
    }
    p = delim + 1;

    /* priority: empty field means default (0) */
    delim = strchr(p, '|');
    if (!delim) return false;
    int prio = 0;
    if (p != delim) {
        prio = (int)strtol(p, NULL, 10);
    }
    p = delim + 1;

    /* persistence in deciseconds (tenths of a second), clamped 1-600 */
    int persist = (int)strtol(p, NULL, 10);
    if (persist < 1)   persist = 1;
    if (persist > 600) persist = 600;

    out->r = (uint8_t)(r & 0xFF);
    out->g = (uint8_t)(g & 0xFF);
    out->b = (uint8_t)(b & 0xFF);
    out->animation_type      = (uint8_t)(anim & 0xFF);
    out->brightness_override = brightness;
    out->priority            = prio;
    out->persistence_ds      = persist;

    return true;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  iBeacon payload parser
 *
 *  Detects iBeacon advertisements whose UUID matches CREATOR_IBEACON_UUID.
 *  Major = (R << 8) | G,  Minor = (B << 8) | (anim << 4 | brightness).
 *  Brightness nibble 0xF means no override.
 * ═════════════════════════════════════════════════════════════════════════════ */

static bool parse_ibeacon_payload(const uint8_t *mfg, uint8_t mfg_len,
                                  creator_packet_t *out)
{
    if (mfg_len < IBEACON_MFG_DATA_LEN)              return false;
    if (mfg[0] != 0x4C || mfg[1] != 0x00)            return false;
    if (mfg[2] != 0x02 || mfg[3] != 0x15)            return false;
    if (memcmp(&mfg[4], CREATOR_IBEACON_UUID, 16) != 0) return false;

    uint16_t major = ((uint16_t)mfg[20] << 8) | mfg[21];
    uint16_t minor = ((uint16_t)mfg[22] << 8) | mfg[23];

    out->r                   = (major >> 8) & 0xFF;
    out->g                   =  major       & 0xFF;
    out->b                   = (minor >> 8) & 0xFF;
    out->animation_type      = (minor >> 4) & 0x0F;
    out->brightness_override = (minor & 0x0F) == 0x0F ? -1 : (minor & 0x0F);
    out->priority            = IBEACON_DEFAULT_PRIORITY;
    out->persistence_ds      = IBEACON_DEFAULT_PERSIST_DS;
    return true;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Packet map helpers
 * ═════════════════════════════════════════════════════════════════════════════ */

static int find_mac_slot(const uint8_t mac[MAC_ADDR_LEN])
{
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (s_packet_map[i].valid &&
            memcmp(s_packet_map[i].mac, mac, MAC_ADDR_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

static int find_free_slot(void)
{
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (!s_packet_map[i].valid) return i;
    }
    /* All slots occupied — evict the oldest entry */
    int oldest = 0;
    for (int i = 1; i < MAX_TRACKED_DEVICES; i++) {
        if (s_packet_map[i].received_time_us < s_packet_map[oldest].received_time_us) {
            oldest = i;
        }
    }
    return oldest;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Periodic evaluation — expire stale packets, pick the best, drive LEDs
 * ═════════════════════════════════════════════════════════════════════════════ */

static void evaluate_packets(void)
{
    int64_t now_us = esp_timer_get_time();

    /* ── 1. Remove expired packets ──────────────────────────────────────── */
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (!s_packet_map[i].valid) continue;
        int64_t age_us = now_us - s_packet_map[i].received_time_us;
        int64_t max_us = (int64_t)s_packet_map[i].persistence_ds * 100000LL
                       + (int64_t)EXPIRY_GRACE_MS * 1000LL;
        if (age_us >= max_us) {
            s_packet_map[i].valid = false;
            ESP_LOGD(TAG, "Expired packet slot %d", i);
        }
    }

    /* ── 2. Select the most relevant packet ─────────────────────────────── */
    creator_packet_t *best = NULL;
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        if (!s_packet_map[i].valid) continue;
        if (!best) {
            best = &s_packet_map[i];
            continue;
        }
        if (s_packet_map[i].priority > best->priority) {
            best = &s_packet_map[i];
        } else if (s_packet_map[i].priority == best->priority) {
            if (s_packet_map[i].rssi > best->rssi) {
                best = &s_packet_map[i];
            } else if (s_packet_map[i].rssi == best->rssi) {
                if (s_packet_map[i].received_time_us > best->received_time_us) {
                    best = &s_packet_map[i];
                }
            }
        }
    }

    /* ── 3. Act on the result ───────────────────────────────────────────── */
    if (!best) {
        if (led_controller_stagger_breath_is_active()) {
            led_controller_stagger_breath_stop();
#if VERBOSE_LOGGING
            ESP_LOGI(TAG, "No CR packets — stopped stagger breath");
#endif
        }
        if (led_controller_solid_color_is_active()) {
            led_controller_solid_color_stop();
#if VERBOSE_LOGGING
            ESP_LOGI(TAG, "No CR packets — stopped solid color");
#endif
        }
        return;
    }

    ESP_LOGD(TAG, "Best CR: RGB(%d,%d,%d) anim=%d prio=%d rssi=%d",
             best->r, best->g, best->b,
             best->animation_type, best->priority, best->rssi);

    if (best->animation_type == 1) {
        if (led_controller_solid_color_is_active())
            led_controller_solid_color_stop();
        led_controller_stagger_breath_start(best->r, best->g, best->b);
    } else if (best->animation_type == 2) {
        if (led_controller_stagger_breath_is_active())
            led_controller_stagger_breath_stop();
        led_controller_solid_color_start(best->r, best->g, best->b);
    } else if (best->animation_type == 0) {
        if (led_controller_stagger_breath_is_active())
            led_controller_stagger_breath_stop();
        if (led_controller_solid_color_is_active())
            led_controller_solid_color_stop();
    }

    if (best->brightness_override >= 0) {
        led_controller_set_brightness_idx(best->brightness_override);
    }
}

/* ── Evaluator FreeRTOS task ─────────────────────────────────────────────── */

static void evaluator_task(void *param)
{
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EVAL_INTERVAL_MS));
        xSemaphoreTake(s_packet_mutex, portMAX_DELAY);
        evaluate_packets();
        xSemaphoreGive(s_packet_mutex);
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  BLE scan result callback
 * ═════════════════════════════════════════════════════════════════════════════ */

static int
ble_scan_event(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    const struct ble_gap_disc_desc *desc = &event->disc;
    struct ble_hs_adv_fields fields = {0};

    if (ble_hs_adv_parse_fields(&fields, desc->data, desc->length_data) != 0) {
        return 0;
    }

    if (!fields.mfg_data || fields.mfg_data_len < 4) {
        return 0;
    }

    creator_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    const char *src_label = NULL;

    /* ── Try CR manufacturer-data format first ───────────────────────────── */
    if (fields.mfg_data_len >= (2 + BLE_TAG_PREFIX_LEN)) {
        const char *tag     = (const char *)&fields.mfg_data[2];
        int         tag_len = fields.mfg_data_len - 2;

        if (memcmp(tag, BLE_TAG_PREFIX, BLE_TAG_PREFIX_LEN) == 0) {
            if (!parse_cr_payload(tag, tag_len, &pkt)) {
                ESP_LOGW(TAG, "Malformed CR payload: \"%.*s\"", tag_len, tag);
                return 0;
            }
            src_label = "CR";
        }
    }

    /* ── Fall back to iBeacon with creator UUID ──────────────────────────── */
    if (!src_label &&
        parse_ibeacon_payload(fields.mfg_data, fields.mfg_data_len, &pkt)) {
        src_label = "iB";
    }

    if (!src_label) return 0;

    memcpy(pkt.mac, desc->addr.val, MAC_ADDR_LEN);
    pkt.rssi             = desc->rssi;
    pkt.received_time_us = esp_timer_get_time();
    pkt.valid            = true;

#if VERBOSE_LOGGING
    const uint8_t *a = desc->addr.val;
    ESP_LOGI(TAG, "%s %02X:%02X:%02X:%02X:%02X:%02X  "
             "RGB(%d,%d,%d) anim=%d bri=%d prio=%d per=%dds rssi=%d",
             src_label,
             a[5], a[4], a[3], a[2], a[1], a[0],
             pkt.r, pkt.g, pkt.b, pkt.animation_type,
             pkt.brightness_override, pkt.priority,
             pkt.persistence_ds, pkt.rssi);
#endif

    xSemaphoreTake(s_packet_mutex, portMAX_DELAY);
    int slot = find_mac_slot(desc->addr.val);
    if (slot < 0) slot = find_free_slot();
    s_packet_map[slot] = pkt;
    xSemaphoreGive(s_packet_mutex);

    return 0;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Start passive BLE scan
 * ═════════════════════════════════════════════════════════════════════════════ */

static void start_scan(void)
{
    struct ble_gap_disc_params scan_params = {
        .itvl          = SCAN_INTERVAL_UNITS,
        .window        = SCAN_WINDOW_UNITS,
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited       = 0,
        .passive        = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &scan_params, ble_scan_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE scan start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE passive scan started (window %d ms / interval %d ms)",
             SCAN_WINDOW_MS, SCAN_INTERVAL_MS);
}

/* ── NimBLE host callbacks ───────────────────────────────────────────────── */

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE ensure addr failed: %d", rc);
        return;
    }
    start_scan();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d — will re-sync", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═════════════════════════════════════════════════════════════════════════════ */

void creator_detector_inject_packet(uint8_t r, uint8_t g, uint8_t b, int persistence_ds)
{
    static const uint8_t fake_mac[MAC_ADDR_LEN] = {0xFE, 0xED, 0x1A, 0x5E, 0x00, 0x00};

    if (persistence_ds < 1) {
        persistence_ds = 1;
    }

    creator_packet_t pkt = {
        .r                  = r,
        .g                  = g,
        .b                  = b,
        .animation_type     = 2,
        .brightness_override = -1,
        .priority           = 6,
        .persistence_ds     = persistence_ds,
        .rssi               = 0,
        .received_time_us   = esp_timer_get_time(),
        .valid              = true,
    };
    memcpy(pkt.mac, fake_mac, MAC_ADDR_LEN);

#if VERBOSE_LOGGING
    ESP_LOGI(TAG, "IR inject CR: RGB(%d,%d,%d) anim=2 prio=6 per=%dds", r, g, b, persistence_ds);
#endif

    xSemaphoreTake(s_packet_mutex, portMAX_DELAY);
    int slot = find_mac_slot(fake_mac);
    if (slot < 0) slot = find_free_slot();
    s_packet_map[slot] = pkt;
    xSemaphoreGive(s_packet_mutex);
}

void creator_detector_start(void)
{
    ESP_LOGI(TAG, "Initializing creator-detector BLE scanner...");

    memset(s_packet_map, 0, sizeof(s_packet_map));
    s_packet_mutex = xSemaphoreCreateMutex();
    configASSERT(s_packet_mutex);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(evaluator_task, "crt_eval", EVAL_TASK_STACK, NULL, 5, NULL);
}
