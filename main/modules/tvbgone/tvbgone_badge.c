/*
 * tvbgone_badge.c
 *
 * Wraps the tv-b-gone ESP-IDF component for the badge firmware.
 * Borrows the lasertag module's IR TX RMT channel, suspends lasertag
 * IR activity for the duration of the sweep, and enforces a 90-second
 * safety timeout.
 */

#include "tvbgone_badge.h"
#include "tvbgone_core.h"
#include "lasertag.h"
#include "libneon_led_controller.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tvbgone";

#define TVBGONE_TIMEOUT_S       120
#define TVBGONE_TIMEOUT_US      ((uint64_t)TVBGONE_TIMEOUT_S * 1000 * 1000)
#define TVBGONE_TIMEOUT_MS      (TVBGONE_TIMEOUT_S * 1000)
#define TVBGONE_TASK_STACK      4096
#define TVBGONE_PROGRESS_POLL_US 500000   /* poll status every 500 ms */

static volatile bool      s_running        = false;
static TaskHandle_t       s_task           = NULL;
static esp_timer_handle_t s_timeout_timer  = NULL;
static esp_timer_handle_t s_progress_timer = NULL;
static portMUX_TYPE       s_lock           = portMUX_INITIALIZER_UNLOCKED;

/* ── 120-second safety timeout ────────────────────────────────────────────── */

static void timeout_cb(void *arg)
{
    ESP_LOGW(TAG, "TV-B-Gone timed out (120 s) — forcing stop");
    tvbgone_core_stop();
}

/* ── Periodic progress poll (feeds tvbgone_core status to LED bar) ───────── */

static void progress_poll_cb(void *arg)
{
    tvbgone_core_status_t st;
    if (tvbgone_core_get_status(&st) == ESP_OK && st.total_codes > 0) {
        led_controller_tvbgone_progress_update(st.current_code_number,
                                                st.total_codes);
    }
}

/* ── Sweep task (runs tvbgone_core_send synchronously) ───────────────────── */

static void tvbgone_send_task(void *arg)
{
    int64_t start_us = esp_timer_get_time();

    ESP_LOGI(TAG, "TV-B-Gone started (both regions, single sweep)");

    /* Suspend lasertag IR activity and start progress animation */
    lasertag_suppress_fire(true);
    temporarily_disable_lasertag_hit_listening(TVBGONE_TIMEOUT_MS);
    led_controller_tvbgone_progress_start();

    /* Arm the safety timeout and start progress polling */
    esp_timer_start_once(s_timeout_timer, TVBGONE_TIMEOUT_US);
    esp_timer_start_periodic(s_progress_timer, TVBGONE_PROGRESS_POLL_US);

    /* Synchronous send — blocks until complete, stopped, or error */
    esp_err_t err = tvbgone_core_send(TVBGONE_CORE_REGION_BOTH,
                                       TVBGONE_CORE_SEND_MODE_SINGLE);

    esp_timer_stop(s_progress_timer);
    esp_timer_stop(s_timeout_timer);          /* no-op if already fired */

    int64_t duration_ms = (esp_timer_get_time() - start_us) / 1000;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TV-B-Gone finished successfully (%lld ms)",
                 (long long)duration_ms);
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "TV-B-Gone stopped (%lld ms)",
                 (long long)duration_ms);
    } else {
        ESP_LOGE(TAG, "TV-B-Gone error: %s (%lld ms)",
                 esp_err_to_name(err), (long long)duration_ms);
    }

    /* Restore lasertag IR state and stop progress animation */
    led_controller_tvbgone_progress_stop();
    lasertag_restore_ir_tx_carrier();
    lasertag_suppress_fire(false);
    re_enable_lasertag_hit_listening();

    taskENTER_CRITICAL(&s_lock);
    s_running = false;
    s_task    = NULL;
    taskEXIT_CRITICAL(&s_lock);

    vTaskDelete(NULL);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void tvbgone_badge_init(void)
{
    tvbgone_core_config_t cfg;
    tvbgone_core_get_default_config(&cfg);
    cfg.rmt_channel_mode     = TVBGONE_CORE_RMT_CHANNEL_MODE_BORROWED;
    cfg.external_rmt_channel = lasertag_get_ir_tx_channel();
    ESP_ERROR_CHECK(tvbgone_core_init(&cfg));

    const esp_timer_create_args_t timeout_args = {
        .callback = timeout_cb,
        .name     = "tvbg_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timeout_args, &s_timeout_timer));

    const esp_timer_create_args_t progress_args = {
        .callback = progress_poll_cb,
        .name     = "tvbg_progress",
    };
    ESP_ERROR_CHECK(esp_timer_create(&progress_args, &s_progress_timer));

    ESP_LOGI(TAG, "TV-B-Gone badge module initialized");
}

esp_err_t tvbgone_badge_start(void)
{
    bool already;

    taskENTER_CRITICAL(&s_lock);
    already = s_running;
    if (!already) s_running = true;
    taskEXIT_CRITICAL(&s_lock);

    if (already) return ESP_ERR_INVALID_STATE;

    BaseType_t ok = xTaskCreate(tvbgone_send_task, "tvbgone",
                                TVBGONE_TASK_STACK, NULL, 5, &s_task);
    if (ok != pdPASS) {
        taskENTER_CRITICAL(&s_lock);
        s_running = false;
        taskEXIT_CRITICAL(&s_lock);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void tvbgone_badge_stop(void)
{
    bool running;

    taskENTER_CRITICAL(&s_lock);
    running = s_running;
    taskEXIT_CRITICAL(&s_lock);

    if (!running) return;

    tvbgone_core_stop();
}

bool tvbgone_badge_is_running(void)
{
    return s_running;
}
