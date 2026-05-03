/*
 * lasertag_main.c
 *
 * Two or more devices running this firmware can play laser tag:
 *   - Press the fire button to shoot (sends an IR laser tag packet)
 *   - When hit, the device logs the attacker info and is disabled
 *     for DISABLE_TIME_AFTER_TAG_MS milliseconds
 *   - Rate limiting prevents firing faster than once per FIRE_RATE_LIMIT_MS
 *
 * Hardware:
 *   - IR LED / emitter on IR_TX_GPIO_NUM
 *   - IR receiver module on IR_RX_GPIO_NUM
 *   - Fire button handled externally via lasertag_fire_button_pressed()
 *
 * IR code on the Espressif ESP-IDF RMT NEC transceiver example and compatible
 * with the OpenLASIR protocol: https://github.com/danielweidman/OpenLASIR
 */

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_openlasir_encoder.h"
#include "libneon_led_controller.h"
#include "openlasir_utils.h"
#include "sync_string_obf.h"
#include "lasertag.h"
#include "creator_detector.h"
#include "wifi_manager.h"

/* ── Dev / test toggle (short ESP-NOW intervals for bench testing) ───────── */
#define TEST_DEV_MODE  0

/* ── Boot IR self-test: TX color_set_* packets for receiver / LED path testing ─ */
#define TEST_COLOR_SET_TX  0   /* 1 = on boot, TX 5× temporary + 5× permanent, 1 s apart */

/* ── Boot IR self-test: TX laser_tag_fire with random block/device/color ───── */
#define TEST_FIRE_SEND_TX  0   /* 1 = on boot, 5× fire (block 34–64) + 5× (outside), 1 s apart */

/* ── Pin definitions ──────────────────────────────────────────────────────── */
// Ideally we would be using the BSP and initialization would be coming from there. Sorry.
#define IR_TX_GPIO_NUM          19
#define IR_RX_GPIO_NUM         18
#define VIBRATION_MOTOR_GPIO    22

/* ── RMT configuration ────────────────────────────────────────────────────── */
#define IR_RESOLUTION_HZ        1000000   // 1 MHz → 1 tick = 1 us
#define IR_DECODE_MARGIN        200       // tolerance in us for symbol parsing

/* ── OpenLASIR / NEC timing (identical on the wire) ───────────────────────── */
#define LEADING_CODE_DURATION_0  9000
#define LEADING_CODE_DURATION_1  4500
#define PAYLOAD_ZERO_DURATION_0  560
#define PAYLOAD_ZERO_DURATION_1  560
#define PAYLOAD_ONE_DURATION_0   560
#define PAYLOAD_ONE_DURATION_1   1690

/* ── Player / device configuration (loaded from NVS or randomly generated) ── */
static uint8_t  s_my_block_id;
static uint8_t  s_my_device_id;
static uint8_t  s_my_color;
static char     s_my_uuid[7];   /* 6 base32 chars + null terminator */
static uint8_t  s_my_mac[6];    /* STA MAC – populated after WiFi init */

static const char     BASE32_ALPHABET[]  = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
/* Colors eligible for random assignment on first launch.
 * White, Red, and Green are intentionally excluded here since they have
 * other implications (green=success, red=error, white=not a normal color really) */
static const uint8_t  ALLOWED_COLORS[]   = {
    OPENLASIR_COLOR_CYAN,
    OPENLASIR_COLOR_MAGENTA,
    OPENLASIR_COLOR_YELLOW,
    OPENLASIR_COLOR_BLUE,
    OPENLASIR_COLOR_ORANGE,
};
#define NUM_ALLOWED_COLORS  (sizeof(ALLOWED_COLORS) / sizeof(ALLOWED_COLORS[0]))

/* ── Logging ──────────────────────────────────────────────────────────────── */
#define VERBOSE_LOGGING             0   /* 1 = show ESP-NOW hop + wifi PHY chatter */

/* ── Game parameters ──────────────────────────────────────────────────────── */
#define FIRE_RATE_LIMIT_MS          1200
#define DISABLE_TIME_AFTER_TAG_MS   2000
#define VIBRATION_FIRE_MS           300
#define VIBRATION_HIT_MS            1200
#define UPTIME_SAVE_INTERVAL_MS     30000
#define MAX_HIT_RECORDS             150

#if TEST_DEV_MODE
#define ESPNOW_SEND_MIN_MS          250   /* TODO: min 20 s between sync packets */
#define ESPNOW_SEND_MAX_MS          250   /* TODO: max 30 s between sync packets */
#define ESPNOW_HOP_MIN_MS           1000  /* TODO: min 15 (15 * 60 * 1000) between channel hops */
#define ESPNOW_HOP_MAX_MS           1000  /* TODO: max 25 (25 * 60 * 1000) between channel hops */
#define HIT_COOLDOWN_S              3   /* TODO: 15 minutes */
#else
#define ESPNOW_SEND_MIN_MS          10000 /* 10 s between sync packets */
#define ESPNOW_SEND_MAX_MS          20000 /* 20 s between sync packets */
#define ESPNOW_HOP_MIN_MS           (15 * 60 * 1000)   /* min 15 mins between channel hops */
#define ESPNOW_HOP_MAX_MS           (25 * 60 * 1000)   /* max 25 mins between channel hops */
#define HIT_COOLDOWN_S              900   /* 15 minutes */
#endif

typedef struct {
    uint8_t channel;
    bool    use_lr;
} espnow_channel_config_t;

static const espnow_channel_config_t s_channel_configs[] = {
    { .channel = 1,  .use_lr = false },   /* standard 11b on ch 1 */
    { .channel = 11, .use_lr = true  },   /* LR 250 Kbps on ch 11 */
};
#define NUM_CHANNEL_CONFIGS  (sizeof(s_channel_configs) / sizeof(s_channel_configs[0]))

static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const char *TAG = "lasertag";

/* ── Game state ───────────────────────────────────────────────────────────── */
static int64_t  s_last_fire_time_ms = 0;
static int64_t  s_re_enable_at_ms   = 0;
static bool     s_is_disabled       = false;

/* ── External hit-listening suspension (see temporarily_disable / re_enable) ─ */
static volatile bool    s_hit_listening_suspended    = false;
static volatile int64_t s_hit_listening_resume_at_ms = 0;

/* ── Fire suppression (used by TV-B-Gone while it owns the TX channel) ───── */
static volatile bool    s_fire_suppressed = false;

/* ── Persistent uptime (total seconds powered on, survives reboots) ───────── */
static uint32_t s_uptime_offset_s = 0;   /* accumulated from prior sessions */
static int64_t  s_boot_time_us    = 0;   /* esp_timer snapshot taken at init */

static uint32_t get_uptime_s(void)
{
    int64_t session_us = esp_timer_get_time() - s_boot_time_us;
    return s_uptime_offset_s + (uint32_t)(session_us / 1000000);
}

/* ── Per-opponent hit tracker (persisted to NVS alongside uptime) ─────────── */
typedef struct {
    uint8_t  block_id;
    uint8_t  device_id;
    uint8_t  color;
    bool     occupied;
    uint32_t last_hit_s;                 /* uptime when last "first hit" was recorded */
    uint16_t total_hits;                 /* lifetime count of first-in-15-min hits */
} hit_record_t;

/* Compact on-disk layout — no padding, 9 bytes per entry */
typedef struct __attribute__((packed)) {
    uint8_t  block_id;
    uint8_t  device_id;
    uint8_t  color;
    uint32_t last_hit_s;
    uint16_t total_hits;
} hit_record_packed_t;

static hit_record_t s_hit_table[MAX_HIT_RECORDS];
static int          s_hit_table_count = 0;
static bool         s_hit_table_dirty = false;

/* ── Aggregate stats (persisted to NVS) ───────────────────────────────────── */
static uint32_t s_total_counted_hits  = 0;       /* lifetime first-in-15-min hits */
static bool     s_counted_hits_dirty  = false;

static uint32_t s_total_shots_fired   = 0;
static bool     s_shots_fired_dirty   = false;

/* ── Per-color lifetime hit totals (persisted independently of hit table) ── */
static uint32_t s_color_hits_34_64[OPENLASIR_NUM_COLORS] = {0};
static uint32_t s_color_hits_other[OPENLASIR_NUM_COLORS] = {0};
static bool     s_color_hits_dirty = false;

/* ── Hardware handles ─────────────────────────────────────────────────────── */
static rmt_channel_handle_t s_rx_channel = NULL;
static rmt_channel_handle_t s_tx_channel = NULL;
static rmt_encoder_handle_t s_ir_encoder = NULL;
static QueueHandle_t        s_receive_queue = NULL;

/* ── Vibration motor ──────────────────────────────────────────────────────── */
static esp_timer_handle_t s_vibration_timer = NULL;

static void vibration_timer_cb(void *arg)
{
    gpio_set_level(VIBRATION_MOTOR_GPIO, 1);
}

static void vibrate_ms(uint32_t duration_ms)
{
    esp_timer_stop(s_vibration_timer);
    gpio_set_level(VIBRATION_MOTOR_GPIO, 0);
    esp_timer_start_once(s_vibration_timer, (uint64_t)duration_ms * 1000);
}

/* ── Pre-encoded fire packet ──────────────────────────────────────────────── */
static ir_openlasir_scan_code_t s_fire_scan_code;

/* ── Fire request flag (set by button callback, consumed by game loop) ───── */
static volatile bool s_fire_requested = false;

/* ── Receive buffer & configs ─────────────────────────────────────────────── */
static rmt_symbol_word_t     s_raw_symbols[64];
static rmt_receive_config_t  s_receive_config;
static rmt_transmit_config_t s_transmit_config;

/* ═════════════════════════════════════════════════════════════════════════════
 *  IR frame parsing (OpenLASIR wire format — same timing as NEC)
 * ═════════════════════════════════════════════════════════════════════════════ */

static inline bool check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IR_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_DECODE_MARGIN));
}

static inline bool parse_logic0(rmt_symbol_word_t *sym)
{
    return check_in_range(sym->duration0, PAYLOAD_ZERO_DURATION_0) &&
           check_in_range(sym->duration1, PAYLOAD_ZERO_DURATION_1);
}

static inline bool parse_logic1(rmt_symbol_word_t *sym)
{
    return check_in_range(sym->duration0, PAYLOAD_ONE_DURATION_0) &&
           check_in_range(sym->duration1, PAYLOAD_ONE_DURATION_1);
}

/**
 * Parse a received RMT symbol stream into a validated OpenLASIR packet.
 * Returns true on success with the decoded packet written to *out_pkt.
 */
static bool parse_openlasir_frame(rmt_symbol_word_t *symbols, size_t num_symbols,
                                  openlasir_packet_t *out_pkt)
{
    if (num_symbols != 34) {
        ESP_LOGD(TAG, "RX parse: expected 34 symbols, got %d", (int)num_symbols);
        return false;
    }

    rmt_symbol_word_t *cur = symbols;

    if (!check_in_range(cur->duration0, LEADING_CODE_DURATION_0) ||
        !check_in_range(cur->duration1, LEADING_CODE_DURATION_1)) {
        ESP_LOGD(TAG, "RX parse: bad leading code (%lu, %lu)",
                 (unsigned long)cur->duration0, (unsigned long)cur->duration1);
        return false;
    }
    cur++;

    uint16_t address = 0;
    for (int i = 0; i < 16; i++) {
        if (parse_logic1(cur)) {
            address |= 1 << i;
        } else if (!parse_logic0(cur)) {
            ESP_LOGD(TAG, "RX parse: bad address bit %d", i);
            return false;
        }
        cur++;
    }

    uint16_t command = 0;
    for (int i = 0; i < 16; i++) {
        if (parse_logic1(cur)) {
            command |= 1 << i;
        } else if (!parse_logic0(cur)) {
            ESP_LOGD(TAG, "RX parse: bad command bit %d", i);
            return false;
        }
        cur++;
    }

    /* OpenLASIR address validation: low byte XOR high byte must equal 0xFF */
    uint8_t addr_lo = address & 0xFF;
    uint8_t addr_hi = (address >> 8) & 0xFF;
    if (addr_lo != (addr_hi ^ 0xFF)) {
        ESP_LOGD(TAG, "RX parse: address check failed (lo=0x%02X hi=0x%02X)", addr_lo, addr_hi);
        return false;
    }

    *out_pkt = openlasir_decode_general_packet(addr_lo, command);
    return true;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  RMT RX callback
 * ═════════════════════════════════════════════════════════════════════════════ */

static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Helpers
 * ═════════════════════════════════════════════════════════════════════════════ */

static void start_receiving(void)
{
    ESP_ERROR_CHECK(rmt_receive(s_rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &s_receive_config));
}

static void flush_rx_queue(void)
{
    rmt_rx_done_event_data_t dummy;
    while (xQueueReceive(s_receive_queue, &dummy, 0) == pdPASS) {
    }
}

static void fire_laser(int64_t now_ms)
{
    bool rx_active = !s_hit_listening_suspended;

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_disable(s_rx_channel));
        flush_rx_queue();
    }

    ESP_LOGI(TAG, "<< FIRE!  addr=0x%02X cmd=0x%04X",
             (unsigned)(s_fire_scan_code.address & 0xFF),
             (unsigned)s_fire_scan_code.command);

    ESP_ERROR_CHECK(rmt_transmit(s_tx_channel, s_ir_encoder,
                                 &s_fire_scan_code, sizeof(s_fire_scan_code),
                                 &s_transmit_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_channel, portMAX_DELAY));

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
        start_receiving();
    }

    s_last_fire_time_ms = now_ms;
    s_total_shots_fired++;
    s_shots_fired_dirty = true;
}

#if TEST_COLOR_SET_TX
/* ═════════════════════════════════════════════════════════════════════════════
 *  TEST_COLOR_SET_TX — boot-time OpenLASIR color_set_* IR transmissions
 *
 *  Not compiled when TEST_COLOR_SET_TX is 0.
 * ═════════════════════════════════════════════════════════════════════════════ */

#define TEST_COLOR_SET_TX_GAP_MS  1000

static void test_color_set_tx_transmit_one(uint8_t mode, uint8_t color_data)
{
    bool rx_active = !s_hit_listening_suspended;
    uint8_t  addr;
    uint16_t cmd;

    openlasir_encode_general_packet(s_my_block_id, s_my_device_id, mode, color_data,
                                    &addr, &cmd);
    ir_openlasir_scan_code_t scan = ir_openlasir_make_scan_code(addr, cmd);

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_disable(s_rx_channel));
        flush_rx_queue();
    }

    ESP_LOGW(TAG, "[TEST_COLOR_SET_TX] TX %s color=%s (addr=0x%02X cmd=0x%04X)",
             openlasir_get_mode_name(mode), openlasir_get_color_name(color_data),
             (unsigned)(scan.address & 0xFF), (unsigned)scan.command);

    ESP_ERROR_CHECK(rmt_transmit(s_tx_channel, s_ir_encoder,
                                 &scan, sizeof(scan), &s_transmit_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_channel, portMAX_DELAY));

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
        start_receiving();
    }
}

static void test_color_set_tx_run_boot_sequence(void)
{
    ESP_LOGW(TAG, "[TEST_COLOR_SET_TX] Boot: 5× color_set_temporary, then 5× "
                  "color_set_permanent (%d ms between packets)",
             TEST_COLOR_SET_TX_GAP_MS);

    for (int i = 0; i < 5; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(TEST_COLOR_SET_TX_GAP_MS));
        }
        test_color_set_tx_transmit_one(OPENLASIR_MODE_COLOR_SET_TEMPORARY,
                                       (uint8_t)(esp_random() % OPENLASIR_NUM_COLORS));
    }
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(TEST_COLOR_SET_TX_GAP_MS));
        test_color_set_tx_transmit_one(OPENLASIR_MODE_COLOR_SET_PERMANENT,
                                       (uint8_t)(esp_random() % OPENLASIR_NUM_COLORS));
    }

    ESP_LOGW(TAG, "[TEST_COLOR_SET_TX] Boot sequence finished.");
}
#endif /* TEST_COLOR_SET_TX */

#if TEST_FIRE_SEND_TX
/* ═════════════════════════════════════════════════════════════════════════════
 *  TEST_FIRE_SEND_TX — boot-time OpenLASIR laser_tag_fire IR transmissions
 *
 *  Not compiled when TEST_FIRE_SEND_TX is 0.
 * ═════════════════════════════════════════════════════════════════════════════ */

#define TEST_FIRE_SEND_TX_GAP_MS       1000
#define TEST_FIRE_SEND_TX_BLOCK_MIN    34
#define TEST_FIRE_SEND_TX_BLOCK_MAX   64

static void test_fire_send_tx_transmit_one(uint8_t block_id, uint8_t device_id, uint8_t color)
{
    bool rx_active = !s_hit_listening_suspended;
    uint8_t  addr;
    uint16_t cmd;

    openlasir_encode_laser_tag_fire(block_id, device_id, color, &addr, &cmd);
    ir_openlasir_scan_code_t scan = ir_openlasir_make_scan_code(addr, cmd);

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_disable(s_rx_channel));
        flush_rx_queue();
    }

    ESP_LOGW(TAG, "[TEST_FIRE_SEND_TX] TX laser_tag_fire block=%u dev=%u color=%s "
                  "(addr=0x%02X cmd=0x%04X)",
             (unsigned)block_id, (unsigned)device_id, openlasir_get_color_name(color),
             (unsigned)(scan.address & 0xFF), (unsigned)scan.command);

    ESP_ERROR_CHECK(rmt_transmit(s_tx_channel, s_ir_encoder,
                                 &scan, sizeof(scan), &s_transmit_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_channel, portMAX_DELAY));

    if (rx_active) {
        ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
        start_receiving();
    }
}

static uint8_t test_fire_send_tx_random_block_inside_34_64(void)
{
    uint32_t span = (uint32_t)(TEST_FIRE_SEND_TX_BLOCK_MAX - TEST_FIRE_SEND_TX_BLOCK_MIN + 1);
    return (uint8_t)(TEST_FIRE_SEND_TX_BLOCK_MIN + (esp_random() % span));
}

static uint8_t test_fire_send_tx_random_block_outside_34_64(void)
{
    uint8_t b;
    do {
        b = (uint8_t)(esp_random() & 0xFF);
    } while (b >= TEST_FIRE_SEND_TX_BLOCK_MIN && b <= TEST_FIRE_SEND_TX_BLOCK_MAX);
    return b;
}

static void test_fire_send_tx_run_boot_sequence(void)
{
    ESP_LOGW(TAG, "[TEST_FIRE_SEND_TX] Boot: 5× fire (block %d–%d), then 5× fire "
                  "(block outside that range) (%d ms between packets)",
             TEST_FIRE_SEND_TX_BLOCK_MIN, TEST_FIRE_SEND_TX_BLOCK_MAX,
             TEST_FIRE_SEND_TX_GAP_MS);

    for (int i = 0; i < 5; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(TEST_FIRE_SEND_TX_GAP_MS));
        }
        test_fire_send_tx_transmit_one(
            test_fire_send_tx_random_block_inside_34_64(),
            (uint8_t)(esp_random() & 0xFF),
            (uint8_t)(esp_random() % OPENLASIR_NUM_COLORS));
    }
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(TEST_FIRE_SEND_TX_GAP_MS));
        test_fire_send_tx_transmit_one(
            test_fire_send_tx_random_block_outside_34_64(),
            (uint8_t)(esp_random() & 0xFF),
            (uint8_t)(esp_random() % OPENLASIR_NUM_COLORS));
    }

    ESP_LOGW(TAG, "[TEST_FIRE_SEND_TX] Boot sequence finished.");
}
#endif /* TEST_FIRE_SEND_TX */

/* ═════════════════════════════════════════════════════════════════════════════
 *  ESP-NOW background broadcast
 * ═════════════════════════════════════════════════════════════════════════════ */

static SemaphoreHandle_t s_espnow_tx_done = NULL;
static int s_espnow_initial_config_idx = 0;

static uint32_t rand_range_ms(uint32_t min_ms, uint32_t max_ms)
{
    return min_ms + (esp_random() % (max_ms - min_ms + 1));
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                            esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW TX cb -> FAIL");
    }
    if (s_espnow_tx_done) {
        xSemaphoreGive(s_espnow_tx_done);
    }
}

static void wifi_init_for_espnow(void)
{
    /* WiFi stack (netif, event loop, esp_wifi_init) already done by wifi_init() */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Let PHY calibration fully settle before reconfiguring */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Force 20 MHz bandwidth for consistent range */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    int8_t actual_power = 0;
    esp_wifi_get_max_tx_power(&actual_power);
    ESP_LOGI(TAG, "TX power: requested=84 (21dBm), actual=%d (%.1fdBm)",
             actual_power, actual_power / 4.0);

    s_espnow_initial_config_idx = esp_random() % NUM_CHANNEL_CONFIGS;

    {
        const espnow_channel_config_t *init_cfg = &s_channel_configs[s_espnow_initial_config_idx];
        if (init_cfg->use_lr) {
            ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
                            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
                            | WIFI_PROTOCOL_LR));
        } else {
            ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
                            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
        }
        ESP_ERROR_CHECK(esp_wifi_set_channel(init_cfg->channel, WIFI_SECOND_CHAN_NONE));
    }

    uint8_t primary_ch = 0;
    wifi_second_chan_t second_ch = 0;
    esp_wifi_get_channel(&primary_ch, &second_ch);
    ESP_LOGI(TAG, "WiFi channel: requested=%d, actual=%d",
             s_channel_configs[s_espnow_initial_config_idx].channel, primary_ch);

    if (!VERBOSE_LOGGING) {
        esp_log_level_set("wifi", ESP_LOG_ERROR);
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    s_espnow_tx_done = xSemaphoreCreateBinary();
    assert(s_espnow_tx_done);

    uint32_t espnow_ver = 0;
    esp_now_get_version(&espnow_ver);
    ESP_LOGI(TAG, "ESP-NOW version: %lu", (unsigned long)espnow_ver);

    /* Channel 0 = "use whatever channel the interface is currently on" */
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    {
        const espnow_channel_config_t *init_cfg = &s_channel_configs[s_espnow_initial_config_idx];
        esp_now_rate_config_t rate_cfg;
        if (init_cfg->use_lr) {
            rate_cfg = (esp_now_rate_config_t){
                .phymode = WIFI_PHY_MODE_LR,
                .rate    = WIFI_PHY_RATE_LORA_250K,
            };
        } else {
            rate_cfg = (esp_now_rate_config_t){
                .phymode = WIFI_PHY_MODE_11B,
                .rate    = WIFI_PHY_RATE_1M_L,
            };
        }
        ESP_ERROR_CHECK(esp_now_set_peer_rate_config(s_broadcast_mac, &rate_cfg));
    }

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_my_mac));
    ESP_LOGI(TAG, "ESP-NOW ready on ch %d (sync_string broadcast)",
             (int)primary_ch);
    ESP_LOGI(TAG, "STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_my_mac[0], s_my_mac[1], s_my_mac[2],
             s_my_mac[3], s_my_mac[4], s_my_mac[5]);
}

static void apply_channel_config(const espnow_channel_config_t *cfg)
{
    if (cfg->use_lr) {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
                        | WIFI_PROTOCOL_LR));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    }

    ESP_ERROR_CHECK(esp_wifi_set_channel(cfg->channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    esp_now_rate_config_t rate_cfg;
    if (cfg->use_lr) {
        rate_cfg = (esp_now_rate_config_t){
            .phymode = WIFI_PHY_MODE_LR,
            .rate    = WIFI_PHY_RATE_LORA_250K,
        };
    } else {
        rate_cfg = (esp_now_rate_config_t){
            .phymode = WIFI_PHY_MODE_11B,
            .rate    = WIFI_PHY_RATE_1M_L,
        };
    }
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(s_broadcast_mac, &rate_cfg));
}

static void espnow_broadcast_task(void *arg)
{
    char buf[_XSO_ML];
    int config_idx = s_espnow_initial_config_idx;
    int64_t last_hop_ms = esp_timer_get_time() / 1000;
    uint32_t next_hop_interval = rand_range_ms(ESPNOW_HOP_MIN_MS, ESPNOW_HOP_MAX_MS);

    /*
     * Track what WE last set the radio to, so we can detect if something
     * external (e.g. a webserver AP) changed the channel/protocol.
     * Initialised to match what wifi_init_for_espnow() applied.
     */
    uint8_t last_applied_channel = s_channel_configs[s_espnow_initial_config_idx].channel;
    bool    last_applied_lr      = s_channel_configs[s_espnow_initial_config_idx].use_lr;

    vTaskDelay(pdMS_TO_TICKS(2000));
    last_hop_ms = esp_timer_get_time() / 1000;

    bool was_sta_connected = false;

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        /*
         * When the WiFi STA is associated with an AP the radio is locked to
         * the AP's channel.  Channel hopping and LR protocol changes are
         * impossible in that state, so we skip them and just send ESP-NOW
         * on whatever channel the AP dictates.
         */
        wifi_ap_record_t ap_info;
        bool sta_connected = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);

        if (sta_connected && !was_sta_connected) {
            ESP_LOGW(TAG, "STA connected to AP — ESP-NOW hopping paused "
                     "(sending on AP channel %d only)", (int)ap_info.primary);
        } else if (!sta_connected && was_sta_connected) {
            ESP_LOGI(TAG, "STA disconnected — resuming ESP-NOW channel hopping");
            last_hop_ms = now_ms;
            next_hop_interval = rand_range_ms(ESPNOW_HOP_MIN_MS, ESPNOW_HOP_MAX_MS);
        }
        was_sta_connected = sta_connected;

        /* Skip all radio work while wifi_manager owns the radio for a scan */
        if (wifi_is_scanning()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        bool external_change = false;
        uint8_t saved_ch = 0;
        wifi_second_chan_t saved_second = 0;
        uint8_t saved_proto = 0;

        if (!sta_connected) {
            if ((now_ms - last_hop_ms) >= (int64_t)next_hop_interval) {
                config_idx = (config_idx + 1) % NUM_CHANNEL_CONFIGS;
                last_hop_ms = now_ms;
                next_hop_interval = rand_range_ms(ESPNOW_HOP_MIN_MS, ESPNOW_HOP_MAX_MS);
                if (VERBOSE_LOGGING) {
                    ESP_LOGI(TAG, "ESP-NOW desired config hop -> ch %d, LR %s (next hop in %lu min)",
                             s_channel_configs[config_idx].channel,
                             s_channel_configs[config_idx].use_lr ? "ON" : "OFF",
                             (unsigned long)(next_hop_interval / 60000));
                }
            }

            const espnow_channel_config_t *desired = &s_channel_configs[config_idx];

            /* ── Snapshot current radio state ─────────────────────────────── */
            esp_wifi_get_channel(&saved_ch, &saved_second);
            esp_wifi_get_protocol(WIFI_IF_STA, &saved_proto);
            bool actual_lr = (saved_proto & WIFI_PROTOCOL_LR) != 0;

            external_change = (saved_ch != last_applied_channel)
                           || (actual_lr != last_applied_lr);

            bool need_reconfig = external_change
                              || (saved_ch != desired->channel)
                              || (actual_lr != desired->use_lr);

            if (need_reconfig) {
                apply_channel_config(desired);
                vTaskDelay(pdMS_TO_TICKS(25));
            }
        }

        /* ── Build and transmit sync string ───────────────────────────────── */
        _xso_p_t sync_params = {
            ._a = s_my_uuid,
            ._b = s_my_block_id,
            ._c = s_my_device_id,
            ._d = s_my_color,
            ._e = s_color_hits_34_64,
            ._f = s_color_hits_other,
            ._g = s_total_shots_fired,
            ._h = get_uptime_s(),
            ._i = (uint32_t)((esp_timer_get_time() - s_boot_time_us) / 1000000),
            ._j = s_my_mac,
        };
        int len = _xso_b(&sync_params, buf, sizeof(buf));
        if (len > 0 && len < (int)sizeof(buf)) {
            esp_err_t ret = esp_now_send(s_broadcast_mac, (uint8_t *)buf, len);
            if (ret == ESP_OK) {
                xSemaphoreTake(s_espnow_tx_done, pdMS_TO_TICKS(100));
            } else {
                ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(ret));
            }
        }

        /* ── Restore radio if external code had changed it ────────────────── */
        if (!sta_connected) {
            const espnow_channel_config_t *desired = &s_channel_configs[config_idx];
            if (external_change) {
                esp_wifi_set_protocol(WIFI_IF_STA, saved_proto);
                esp_wifi_set_channel(saved_ch, saved_second);
                ESP_LOGD(TAG, "ESP-NOW TX on ch %d LR=%s (restored ch %d)",
                         desired->channel, desired->use_lr ? "ON" : "OFF",
                         saved_ch);
            } else {
                last_applied_channel = desired->channel;
                last_applied_lr      = desired->use_lr;
                ESP_LOGD(TAG, "ESP-NOW TX on ch %d LR=%s",
                         desired->channel, desired->use_lr ? "ON" : "OFF");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(rand_range_ms(ESPNOW_SEND_MIN_MS, ESPNOW_SEND_MAX_MS)));
    }
}
/* ═════════════════════════════════════════════════════════════════════════════
 *  Player config: load from NVS or generate + persist on first boot
 * ═════════════════════════════════════════════════════════════════════════════ */

void lasertag_init_player_config(void)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("player", NVS_READWRITE, &nvs));

    uint8_t block_id, device_id, color;
    char    uuid[7] = {0};
    size_t  uuid_len = sizeof(uuid);

    bool have_config = (nvs_get_u8(nvs, "block_id",  &block_id)  == ESP_OK)
                    && (nvs_get_u8(nvs, "device_id", &device_id) == ESP_OK)
                    && (nvs_get_u8(nvs, "color",     &color)     == ESP_OK)
                    && (nvs_get_str(nvs, "uuid", uuid, &uuid_len) == ESP_OK);

    if (have_config) {
        s_my_block_id  = block_id;
        s_my_device_id = device_id;
        s_my_color     = color;
        memcpy(s_my_uuid, uuid, sizeof(s_my_uuid));
        ESP_LOGI(TAG, "Loaded player config from NVS");
    } else {
        s_my_block_id  = 35 + (esp_random() % 30);   /* 35 .. 64 inclusive */
        s_my_device_id = esp_random() & 0xFF;
        s_my_color     = ALLOWED_COLORS[esp_random() % NUM_ALLOWED_COLORS];

        for (int i = 0; i < 6; i++) {
            s_my_uuid[i] = BASE32_ALPHABET[esp_random() % 32];
        }
        s_my_uuid[6] = '\0';

        ESP_ERROR_CHECK(nvs_set_u8(nvs,  "block_id",  s_my_block_id));
        ESP_ERROR_CHECK(nvs_set_u8(nvs,  "device_id", s_my_device_id));
        ESP_ERROR_CHECK(nvs_set_u8(nvs,  "color",     s_my_color));
        ESP_ERROR_CHECK(nvs_set_str(nvs,  "uuid",     s_my_uuid));
        ESP_ERROR_CHECK(nvs_commit(nvs));
        ESP_LOGI(TAG, "Generated and saved new player config to NVS");
    }

    nvs_close(nvs);

    ESP_LOGI(TAG, "Player: block=%d device=%d color=%s uuid=%s",
             s_my_block_id, s_my_device_id,
             openlasir_get_color_name(s_my_color), s_my_uuid);
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Persistent state: load / save
 * ═════════════════════════════════════════════════════════════════════════════ */

static void load_hit_table(nvs_handle_t nvs)
{
    size_t blob_len = 0;
    if (nvs_get_blob(nvs, "hits", NULL, &blob_len) != ESP_OK || blob_len == 0) {
        s_hit_table_count = 0;
        return;
    }

    int count = (int)(blob_len / sizeof(hit_record_packed_t));
    if (count > MAX_HIT_RECORDS) {
        count = MAX_HIT_RECORDS;
    }

    hit_record_packed_t *packed = malloc(count * sizeof(hit_record_packed_t));
    if (!packed) {
        ESP_LOGW(TAG, "load_hit_table: malloc failed for %d records", count);
        s_hit_table_count = 0;
        return;
    }

    size_t read_len = count * sizeof(hit_record_packed_t);
    if (nvs_get_blob(nvs, "hits", packed, &read_len) != ESP_OK) {
        free(packed);
        s_hit_table_count = 0;
        return;
    }

    for (int i = 0; i < count; i++) {
        s_hit_table[i].block_id   = packed[i].block_id;
        s_hit_table[i].device_id  = packed[i].device_id;
        s_hit_table[i].color      = packed[i].color;
        s_hit_table[i].last_hit_s = packed[i].last_hit_s;
        s_hit_table[i].total_hits = packed[i].total_hits;
        s_hit_table[i].occupied   = true;
    }
    free(packed);
    s_hit_table_count = count;
    ESP_LOGI(TAG, "Loaded %d hit records from NVS", count);
}

static void load_all_state(void)
{
    s_boot_time_us = esp_timer_get_time();

    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("player", NVS_READONLY, &nvs));

    if (nvs_get_u32(nvs, "uptime_s", &s_uptime_offset_s) != ESP_OK)
        s_uptime_offset_s = 0;
    if (nvs_get_u32(nvs, "cnt_hits", &s_total_counted_hits) != ESP_OK)
        s_total_counted_hits = 0;
    if (nvs_get_u32(nvs, "cnt_shots", &s_total_shots_fired) != ESP_OK)
        s_total_shots_fired = 0;

    load_hit_table(nvs);

    {
        size_t len = sizeof(s_color_hits_34_64);
        if (nvs_get_blob(nvs, "ch_34_64", s_color_hits_34_64, &len) != ESP_OK)
            memset(s_color_hits_34_64, 0, sizeof(s_color_hits_34_64));
        len = sizeof(s_color_hits_other);
        if (nvs_get_blob(nvs, "ch_other", s_color_hits_other, &len) != ESP_OK)
            memset(s_color_hits_other, 0, sizeof(s_color_hits_other));
    }

    nvs_close(nvs);

    ESP_LOGI(TAG, "Uptime offset: %lu s  Counted hits: %lu  Shots fired: %lu",
             (unsigned long)s_uptime_offset_s,
             (unsigned long)s_total_counted_hits,
             (unsigned long)s_total_shots_fired);
}

static void save_periodic_state(void)
{
    bool anything_dirty = s_hit_table_dirty || s_counted_hits_dirty
                       || s_shots_fired_dirty || s_color_hits_dirty;

    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("player", NVS_READWRITE, &nvs));

    /* Uptime always advances */
    ESP_ERROR_CHECK(nvs_set_u32(nvs, "uptime_s", get_uptime_s()));

    if (s_hit_table_dirty) {
        hit_record_packed_t *packed = malloc(s_hit_table_count * sizeof(hit_record_packed_t));
        if (packed) {
            int count = 0;
            for (int i = 0; i < s_hit_table_count; i++) {
                if (s_hit_table[i].occupied) {
                    packed[count].block_id   = s_hit_table[i].block_id;
                    packed[count].device_id  = s_hit_table[i].device_id;
                    packed[count].color      = s_hit_table[i].color;
                    packed[count].last_hit_s = s_hit_table[i].last_hit_s;
                    packed[count].total_hits = s_hit_table[i].total_hits;
                    count++;
                }
            }
            ESP_ERROR_CHECK(nvs_set_blob(nvs, "hits", packed,
                                         count * sizeof(hit_record_packed_t)));
            free(packed);
            s_hit_table_dirty = false;
        } else {
            ESP_LOGW(TAG, "save_periodic_state: malloc failed, skipping hit table save");
        }
    }

    if (s_counted_hits_dirty) {
        ESP_ERROR_CHECK(nvs_set_u32(nvs, "cnt_hits", s_total_counted_hits));
        s_counted_hits_dirty = false;
    }

    if (s_shots_fired_dirty) {
        ESP_ERROR_CHECK(nvs_set_u32(nvs, "cnt_shots", s_total_shots_fired));
        s_shots_fired_dirty = false;
    }

    if (s_color_hits_dirty) {
        ESP_ERROR_CHECK(nvs_set_blob(nvs, "ch_34_64", s_color_hits_34_64,
                                     sizeof(s_color_hits_34_64)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs, "ch_other", s_color_hits_other,
                                     sizeof(s_color_hits_other)));
        s_color_hits_dirty = false;
    }

    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);

    if (anything_dirty) {
        ESP_LOGD(TAG, "Periodic save: uptime=%lu s  hits=%lu  shots=%lu",
                 (unsigned long)get_uptime_s(),
                 (unsigned long)s_total_counted_hits,
                 (unsigned long)s_total_shots_fired);
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Hit tracker lookup
 * ═════════════════════════════════════════════════════════════════════════════ */

/**
 * Look up an opponent in the hit table.
 * Returns pointer to existing record, or NULL if not found.
 */
static hit_record_t *hit_table_find(uint8_t block_id, uint8_t device_id, uint8_t color)
{
    for (int i = 0; i < s_hit_table_count; i++) {
        hit_record_t *r = &s_hit_table[i];
        if (r->occupied &&
            r->block_id  == block_id &&
            r->device_id == device_id &&
            r->color     == color) {
            return r;
        }
    }
    return NULL;
}

/**
 * Record a hit. Returns true if this is the first hit from this opponent
 * within HIT_COOLDOWN_S seconds, false if it is a repeat.
 * Only updates the stored timestamp and increments the lifetime counter
 * on a "first" hit.  *out_total receives the opponent's lifetime count.
 */
static bool hit_table_record(uint8_t block_id, uint8_t device_id, uint8_t color,
                              uint16_t *out_total)
{
    uint32_t now_s = get_uptime_s();
    hit_record_t *r = hit_table_find(block_id, device_id, color);

    if (r) {
        if (now_s - r->last_hit_s >= HIT_COOLDOWN_S) {
            r->last_hit_s = now_s;
            r->total_hits++;
            s_hit_table_dirty = true;
            *out_total = r->total_hits;
            return true;
        }
        *out_total = r->total_hits;
        return false;
    }

    /* New opponent — insert into table */
    if (s_hit_table_count < MAX_HIT_RECORDS) {
        r = &s_hit_table[s_hit_table_count++];
    } else {
        /* Table full — evict oldest record */
        r = &s_hit_table[0];
        for (int i = 1; i < MAX_HIT_RECORDS; i++) {
            if (s_hit_table[i].last_hit_s < r->last_hit_s) {
                r = &s_hit_table[i];
            }
        }
    }
    r->block_id   = block_id;
    r->device_id  = device_id;
    r->color      = color;
    r->occupied   = true;
    r->last_hit_s = now_s;
    r->total_hits = 1;
    s_hit_table_dirty = true;
    *out_total = 1;
    return true;
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Public API: fire button trigger
 * ═════════════════════════════════════════════════════════════════════════════ */

void lasertag_fire_button_pressed(void)
{
    s_fire_requested = true;
}

void temporarily_disable_lasertag_hit_listening(int32_t timeout_ms)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (timeout_ms > 0) {
        s_hit_listening_resume_at_ms = now_ms + (int64_t)timeout_ms;
    } else {
        s_hit_listening_resume_at_ms = 0;
    }
    s_hit_listening_suspended = true;
    ESP_LOGI(TAG, "Hit listening suspended (timeout=%ld ms)", (long)timeout_ms);
}

void re_enable_lasertag_hit_listening(void)
{
    s_hit_listening_suspended = false;
    s_hit_listening_resume_at_ms = 0;
    ESP_LOGI(TAG, "Hit listening manually resumed.");
}

rmt_channel_handle_t lasertag_get_ir_tx_channel(void)
{
    return s_tx_channel;
}

void lasertag_suppress_fire(bool suppress)
{
    s_fire_suppressed = suppress;
    if (suppress) {
        s_fire_requested = false;
    }
    ESP_LOGI(TAG, "Fire %s", suppress ? "suppressed" : "enabled");
}

void lasertag_restore_ir_tx_carrier(void)
{
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle   = 0.33,
        .frequency_hz = 38000,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(s_tx_channel, &carrier_cfg));
    ESP_LOGD(TAG, "IR TX carrier restored to 38 kHz / 33%%");
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Lasertag module initialization
 * ═════════════════════════════════════════════════════════════════════════════ */

static void lasertag_task(void *arg);

void lasertag_start(void)
{
    ESP_LOGI(TAG, "══════════════════════════════════════");
    ESP_LOGI(TAG, " OpenLASIR Laser Tag — ESP-IDF");
    ESP_LOGI(TAG, "══════════════════════════════════════");

    /* ── Player config + persistent state (NVS must already be initialized) ── */
    lasertag_init_player_config();
    load_all_state();

    /* ── RMT RX channel ──────────────────────────────────────────────────── */
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = IR_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .gpio_num          = IR_RX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s_rx_channel));

    s_receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(s_receive_queue);

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rx_channel, &cbs, s_receive_queue));

    s_receive_config = (rmt_receive_config_t) {
        .signal_range_min_ns = 1250,      // shortest valid pulse ~560 us >> 1.25 us
        .signal_range_max_ns = 12000000,  // longest valid pulse ~9 ms << 12 ms
    };

    /* ── RMT TX channel ──────────────────────────────────────────────────── */
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = IR_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
        .gpio_num          = IR_TX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle    = 0.33,
        .frequency_hz  = 38000,  // 38 kHz carrier
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(s_tx_channel, &carrier_cfg));

    s_transmit_config = (rmt_transmit_config_t) {
        .loop_count = 0,
    };

    /* ── OpenLASIR encoder ────────────────────────────────────────────────── */
    ir_openlasir_encoder_config_t enc_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_ir_openlasir_encoder(&enc_cfg, &s_ir_encoder));

    /* ── Pre-encode fire packet ───────────────────────────────────────────── */
    {
        uint8_t  addr;
        uint16_t cmd;
        openlasir_encode_laser_tag_fire(s_my_block_id, s_my_device_id, s_my_color, &addr, &cmd);
        s_fire_scan_code = ir_openlasir_make_scan_code(addr, cmd);

        ESP_LOGI(TAG, "Block ID: %d  Device ID: %d  Color: %s  UUID: %s",
                 s_my_block_id, s_my_device_id,
                 openlasir_get_color_name(s_my_color), s_my_uuid);
        ESP_LOGI(TAG, "Fire packet -> address=0x%02X command=0x%04X", addr, (unsigned)cmd);
    }

    ESP_LOGI(TAG, "IR TX pin: %d, IR RX pin: %d", IR_TX_GPIO_NUM, IR_RX_GPIO_NUM);

    /* ── Vibration motor GPIO + one-shot off-timer ─────────────────────────── */
    {
        gpio_config_t vib_cfg = {
            .pin_bit_mask = 1ULL << VIBRATION_MOTOR_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&vib_cfg));
        gpio_set_level(VIBRATION_MOTOR_GPIO, 1);

        const esp_timer_create_args_t vib_timer_args = {
            .callback = vibration_timer_cb,
            .name     = "vibration",
        };
        ESP_ERROR_CHECK(esp_timer_create(&vib_timer_args, &s_vibration_timer));
    }

    ESP_LOGI(TAG, "Ready! Press the fire button to shoot.");

    /* ── Initialise WiFi and start ESP-NOW v1 raw broadcast task ─────────── */
    wifi_init_for_espnow();
    xTaskCreate(espnow_broadcast_task, "espnow_bcast", 4096, NULL, 5, NULL);

    /* ── Enable TX and RX channels ────────────────────────────────────────── */
    ESP_ERROR_CHECK(rmt_enable(s_tx_channel));
    ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
    start_receiving();

#if TEST_COLOR_SET_TX
    test_color_set_tx_run_boot_sequence();
#endif

#if TEST_FIRE_SEND_TX
    test_fire_send_tx_run_boot_sequence();
#endif

    /* ── Spawn game loop task ─────────────────────────────────────────────── */
    xTaskCreate(lasertag_task, "lasertag", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Lasertag game task started.");
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Lasertag game task (main game loop)
 * ═════════════════════════════════════════════════════════════════════════════ */

static void lasertag_task(void *arg)
{
    s_last_fire_time_ms = esp_timer_get_time() / 1000 - FIRE_RATE_LIMIT_MS;

    int64_t s_last_uptime_save_ms = 0;
    bool was_suspended = false;

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        /* ── Persist uptime + hit table to NVS every ~30 s ─────────────────── */
        if ((now_ms - s_last_uptime_save_ms) >= UPTIME_SAVE_INTERVAL_MS) {
            save_periodic_state();
            s_last_uptime_save_ms = now_ms;
        }

        /* ── Handle external hit-listening suspension ──────────────────────── */
        bool currently_suspended = s_hit_listening_suspended;

        if (currently_suspended && !was_suspended) {
            if (!s_is_disabled) {
                ESP_ERROR_CHECK(rmt_disable(s_rx_channel));
                flush_rx_queue();
            }
            ESP_LOGI(TAG, ">> Hit listening suspended by external request.");
        }

        if (currently_suspended && s_hit_listening_resume_at_ms > 0
            && now_ms >= s_hit_listening_resume_at_ms) {
            s_hit_listening_suspended = false;
            s_hit_listening_resume_at_ms = 0;
            currently_suspended = false;
            ESP_LOGI(TAG, ">> Hit listening auto-resumed after timeout.");
        }

        if (!currently_suspended && was_suspended) {
            if (!s_is_disabled) {
                ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
                start_receiving();
            }
            ESP_LOGI(TAG, ">> Hit listening resumed.");
        }

        was_suspended = currently_suspended;

        /* ── Re-enable after tag cooldown ─────────────────────────────────── */
        if (s_is_disabled && now_ms >= s_re_enable_at_ms) {
            s_is_disabled = false;
            if (!s_hit_listening_suspended) {
                ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
                start_receiving();
            }
            if (VERBOSE_LOGGING) {
                ESP_LOGI(TAG, ">> Re-enabled after tag cooldown.");
            }
        }

        /* ── Check for received IR data ───────────────────────────────────── */
        rmt_rx_done_event_data_t rx_data;
        if (!s_is_disabled && !s_hit_listening_suspended
            && xQueueReceive(s_receive_queue, &rx_data, 0) == pdPASS) {
            if (VERBOSE_LOGGING) {
                ESP_LOGI(TAG, "RX: %d symbols received", (int)rx_data.num_symbols);
            }
            openlasir_packet_t pkt;

            if (parse_openlasir_frame(rx_data.received_symbols, rx_data.num_symbols, &pkt)) {
                if (pkt.mode == OPENLASIR_MODE_LASER_TAG_FIRE) {
                    /* ── HIT! ─────────────────────────────────────────────── */
                    uint8_t r = 0, g = 0, b = 0;
                    openlasir_get_color_rgb(pkt.data, &r, &g, &b);

                    uint16_t opp_hits = 0;
                    bool first_hit = hit_table_record(pkt.block_id, pkt.device_id, pkt.data,
                                                      &opp_hits);

                    if (first_hit) {
                        s_total_counted_hits++;
                        s_counted_hits_dirty = true;

                        if (pkt.data < OPENLASIR_NUM_COLORS) {
                            if (pkt.block_id >= 34 && pkt.block_id <= 64) {
                                s_color_hits_34_64[pkt.data]++;
                            } else {
                                s_color_hits_other[pkt.data]++;
                            }
                            s_color_hits_dirty = true;
                        }
                    }

                    if (pkt.data == s_my_color) {
                        if (VERBOSE_LOGGING) {
                            ESP_LOGW(TAG, "!! FRIENDLY FIRE from Block %d Device %d  Color: %s (%d, %d, %d)  [%s]",
                                     pkt.block_id, pkt.device_id,
                                     openlasir_get_color_name(pkt.data), r, g, b,
                                     first_hit ? "FIRST HIT in 15 min" : "REPEAT hit (<15 min)");
                        }
                    } else {
                        if (VERBOSE_LOGGING) {
                            ESP_LOGI(TAG, "!! HIT by ENEMY Block %d Device %d  Color: %s (%d, %d, %d)  [%s]",
                                     pkt.block_id, pkt.device_id,
                                     openlasir_get_color_name(pkt.data), r, g, b,
                                     first_hit ? "FIRST HIT in 15 min" : "REPEAT hit (<15 min)");
                        }
                    }
                    if (VERBOSE_LOGGING) {
                        ESP_LOGI(TAG, "   Opp first-hits: %u  Total counted: %lu  Shots fired: %lu",
                                 (unsigned)opp_hits,
                                 (unsigned long)s_total_counted_hits,
                                 (unsigned long)s_total_shots_fired);
                    }

                    /* LED_TRIGGER: choose hit animation based on team color and first/repeat */
                    bool is_friendly = (pkt.data == s_my_color);
                    if (is_friendly) {
                        led_controller_trigger_hit_friendly(r, g, b);
                    } else if (first_hit) {
                        led_controller_trigger_hit_first(r, g, b);
                    } else {
                        led_controller_trigger_hit_repeat();
                    }

                    if (!is_friendly && first_hit) {
                        vibrate_ms(VIBRATION_HIT_MS);
                        s_is_disabled     = true;
                        s_re_enable_at_ms = now_ms + DISABLE_TIME_AFTER_TAG_MS;

                        ESP_ERROR_CHECK(rmt_disable(s_rx_channel));
                        flush_rx_queue();
                    } else {
                        start_receiving();
                    }
                } else if (pkt.mode == OPENLASIR_MODE_COLOR_SET_TEMPORARY) {
                    uint8_t cr = 0, cg = 0, cb = 0;
                    openlasir_get_color_rgb(pkt.data, &cr, &cg, &cb);
                    if (VERBOSE_LOGGING) {
                        ESP_LOGI(TAG, "color_set_temporary from Block %d Device %d  Color: %s (%d,%d,%d)",
                                 pkt.block_id, pkt.device_id,
                                 openlasir_get_color_name(pkt.data), cr, cg, cb);
                    }
                    creator_detector_inject_packet(cr, cg, cb, 2);
                    start_receiving();
                } else if (pkt.mode == OPENLASIR_MODE_COLOR_SET_PERMANENT) {
                    uint8_t cr = 0, cg = 0, cb = 0;
                    openlasir_get_color_rgb(pkt.data, &cr, &cg, &cb);
                    if (VERBOSE_LOGGING) {
                        ESP_LOGI(TAG, "color_set_permanent from Block %d Device %d  Color: %s (%d,%d,%d)",
                                pkt.block_id, pkt.device_id,
                                openlasir_get_color_name(pkt.data), cr, cg, cb);
                    }

                    /* 3000 ds = 5 minutes (10 ds per second) */
                    creator_detector_inject_packet(cr, cg, cb, 3000);
                    start_receiving();
                } else {
                    ESP_LOGI(TAG, "Received OpenLASIR mode=%s from Block %d Device %d",
                             openlasir_get_mode_name(pkt.mode), pkt.block_id, pkt.device_id);
                    start_receiving();
                }
            } else {
                if (VERBOSE_LOGGING) {
                    ESP_LOGW(TAG, "RX: frame parse failed (%d symbols)", (int)rx_data.num_symbols);
                }
                start_receiving();
            }
        }

        /* ── Fire button (triggered by button component callback) ─────────── */
        if (s_fire_requested && !s_fire_suppressed) {
            s_fire_requested = false;
            if (!s_is_disabled && (now_ms - s_last_fire_time_ms) >= FIRE_RATE_LIMIT_MS) {
                /* LED_TRIGGER: fire — set LED before IR TX so it's visually immediate */
                {
                    uint8_t fr = 0, fg = 0, fb = 0;
                    openlasir_get_color_rgb(s_my_color, &fr, &fg, &fb);
                    led_controller_trigger_fire(fr, fg, fb);
                }
                vibrate_ms(VIBRATION_FIRE_MS);
                fire_laser(now_ms);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
