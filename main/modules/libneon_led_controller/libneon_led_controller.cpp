/*
 * libneon_led_controller.cpp
 *
 * Always-running LED animation module backed by the libneon library.
 * Provides smooth background animations that cycle on demand, and
 * exposes trigger functions so other modules (lasertag) can request
 * one-shot effects that temporarily override the background.
 */

#include "libneon_led_controller.h"

#include <neo/encoder.hpp>
#include <neo/fx.hpp>
#include <neo/alarm.hpp>
#include <neo/channel.hpp>
#include <neo/color.hpp>
#include <neo/gradient.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "openlasir_utils.h"

using namespace std::chrono_literals;
using namespace neo::literals;

static const char *TAG = "led_ctrl";

/* ── Hardware constants ──────────────────────────────────────────────────── */
static constexpr gpio_num_t LED_GPIO       = GPIO_NUM_8;
static constexpr std::size_t NUM_LEDS      = 16;
static constexpr auto        FRAME_RATE    = 80_fps;

/* ── Brightness levels ─────────────────────────────────────────────────────
 *  scale_pixel uses (c * bri) / 255.  255 = 3 * 5 * 17.
 *  If bri *divides* 255, (c*bri)/255 steps by 1 every (255/bri) values of c
 *  (most uniform).  Otherwise use only bri with gcd(bri,255) > 1, i.e. bri
 *  divisible by 3, 5, or 17 — never coprime to 255, which avoids the worst
 *  uneven clumping.  Extra steps are packed toward the low end.
 */
static constexpr uint8_t BRIGHTNESS_LEVELS[] = {
    3,   5,   10,  15,  17,  30,  34,  45,  /* low: divisors + gcd-friendly fill */
    51,  75,  85,  102, 120, 153, 170, 255  /* mid → max */
};
static constexpr int NUM_BRIGHTNESS_LEVELS =
    sizeof(BRIGHTNESS_LEVELS) / sizeof(BRIGHTNESS_LEVELS[0]);
static constexpr int DEFAULT_BRIGHTNESS_IDX = 4;  /* 51, ~ prior default 50 */

static std::atomic<int> s_brightness_idx{DEFAULT_BRIGHTNESS_IDX};

/* ── Suppression (for ESP-NOW radio bursts) ──────────────────────────────── */
static std::atomic<bool> s_suppressed{false};

/* ── Effect priorities (higher number wins) ──────────────────────────────── */
/*  0-3, not doing anything with these yet but I didn't want to start at 0   */
/*  4 = button-rotation background                                           */
/*  5 = stagger breath                                                       */
/*  6 = fire                                                                 */
/*  7 = hit_friendly                                                         */
/*  8 = hit_repeat                                                           */
/*  9 = hit_first                                                            */

/* ── Timed-effect durations ──────────────────────────────────────────────── */
static constexpr int64_t HIT_FIRST_DURATION_MS   = 2000;

/* ── Fire animation (rectangle fill-up → white crown → fade-out) ─────────
 *
 *  Physical LED layout (1-indexed):
 *
 *       1   2   3   4   5          3 = "crown" (white flash)
 *      16               6          Fill rises from bottom-center (11)
 *      15               7          up both sides, meeting at the top.
 *      14               8
 *      13  12  11  10   9
 */
static constexpr int     FIRE_NUM_STEPS = 9;
static constexpr int64_t FIRE_FILL_MS   = 100;
static constexpr int64_t FIRE_HOLD_MS   = 100;
static constexpr int64_t FIRE_FADE_MS   = 800;
static constexpr int64_t FIRE_BLACK_MS  = 250;
static constexpr int64_t FIRE_DURATION_MS = FIRE_FILL_MS + FIRE_HOLD_MS + FIRE_FADE_MS + FIRE_BLACK_MS;

struct fire_step_t {
    uint8_t indices[2];
    uint8_t count;
};

static constexpr fire_step_t FIRE_STEPS[FIRE_NUM_STEPS] = {
    {{10,  0}, 1},   /* step 0 — LED 11              */
    {{11,  9}, 2},   /* step 1 — LEDs 12, 10         */
    {{12,  8}, 2},   /* step 2 — LEDs 13, 9          */
    {{13,  7}, 2},   /* step 3 — LEDs 14, 8          */
    {{14,  6}, 2},   /* step 4 — LEDs 15, 7          */
    {{15,  5}, 2},   /* step 5 — LEDs 16, 6          */
    {{ 0,  4}, 2},   /* step 6 — LEDs 1, 5           */
    {{ 1,  3}, 2},   /* step 7 — LEDs 2, 4  (2× bri) */
    {{ 2,  0}, 1},   /* step 8 — LED 3 white (4× bri) */
};
static constexpr int64_t BLINK_PHASE_MS          = 150;
static constexpr int     BLINK_NUM_CYCLES        = 3;
static constexpr int64_t BLINK_TOTAL_MS          = BLINK_NUM_CYCLES * 2 * BLINK_PHASE_MS;

struct timed_effect_t {
    std::atomic<bool>    active{false};
    std::atomic<int64_t> start_us{0};
    std::atomic<uint8_t> r{0}, g{0}, b{0};
    std::atomic<uint8_t> rand_offset{0};
};

static timed_effect_t s_eff_fire;
static timed_effect_t s_eff_hit_first;
static timed_effect_t s_eff_hit_repeat;
static timed_effect_t s_eff_hit_friendly;

/* ── Stagger breath (API-driven, not in button rotation) ───────────────────── */
static constexpr int64_t STAGGER_BREATH_PERIOD_MS = 3000;
static std::atomic<bool> s_stagger_breath_active{false};
static std::atomic<uint8_t> s_stagger_breath_r{0};
static std::atomic<uint8_t> s_stagger_breath_g{200};
static std::atomic<uint8_t> s_stagger_breath_b{200};

/* ── Solid color fill (API-driven, not in button rotation) ─────────────────── */
static std::atomic<bool>    s_solid_color_active{false};
static std::atomic<uint8_t> s_solid_color_r{0};
static std::atomic<uint8_t> s_solid_color_g{0};
static std::atomic<uint8_t> s_solid_color_b{0};

/* ── TV-B-Gone progress animation (highest priority) ───────────────────────── */
static constexpr int64_t TVBGONE_BLINK_HALF_PERIOD_MS = 500;
static constexpr uint8_t TVBGONE_PURPLE_R = 128;
static constexpr uint8_t TVBGONE_PURPLE_G = 0;
static constexpr uint8_t TVBGONE_PURPLE_B = 255;

static std::atomic<bool>    s_tvbgone_active{false};
static std::atomic<int>     s_tvbgone_solid_count{1};   /* 0..NUM_LEDS */

/* ── Background animation pool ───────────────────────────────────────────── */
static constexpr int NUM_BG_ANIMATIONS = 9;
static std::atomic<bool> s_next_anim_requested{false};

/* ── Pending image palette (set from main task, applied in frame callback) ── */
struct palette_update_t {
    uint8_t r1, g1, b1;
    uint8_t r2, g2, b2;
    uint8_t r3, g3, b3;
};
static palette_update_t  s_pending_palette;
static std::atomic<bool> s_palette_update_requested{false};

/* ── Module-level objects (constructed in led_controller_start) ───────────── */
static neo::led_encoder *s_encoder   = nullptr;
static neo::alarm       *s_alarm     = nullptr;

static std::shared_ptr<neo::transition_fx>  s_bg_transition;
static std::array<std::shared_ptr<neo::fx_base>, NUM_BG_ANIMATIONS> s_bg_pool;
static int s_bg_index = 0;

/* ═════════════════════════════════════════════════════════════════════════════
 *  Helper: scale an srgb pixel by a brightness byte (0-255)
 * ═════════════════════════════════════════════════════════════════════════════ */

static inline neo::srgb scale_pixel(neo::srgb c, uint8_t bri)
{
    return neo::srgb{
        static_cast<uint8_t>((uint16_t(c.r) * bri) / 255),
        static_cast<uint8_t>((uint16_t(c.g) * bri) / 255),
        static_cast<uint8_t>((uint16_t(c.b) * bri) / 255),
    };
}

static void led_transmit(std::vector<neo::srgb> &buf)
{
    ESP_ERROR_CHECK(s_encoder->transmit(buf.begin(), buf.end(),
                                         neo::default_channel_extractor<neo::srgb>{}));
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Per-effect renderers — each returns true while still active
 * ═════════════════════════════════════════════════════════════════════════════ */

static void fill_solid(std::vector<neo::srgb> &buf, uint8_t r, uint8_t g, uint8_t b, uint8_t bri)
{
    neo::srgb c{r, g, b};
    c = scale_pixel(c, bri);
    for (auto &px : buf) px = c;
}

static void activate_effect(timed_effect_t &eff, uint8_t r, uint8_t g, uint8_t b)
{
    eff.r.store(r, std::memory_order_relaxed);
    eff.g.store(g, std::memory_order_relaxed);
    eff.b.store(b, std::memory_order_relaxed);
    eff.rand_offset.store(static_cast<uint8_t>(esp_random() % NUM_LEDS),
                          std::memory_order_relaxed);
    eff.start_us.store(esp_timer_get_time(), std::memory_order_relaxed);
    eff.active.store(true, std::memory_order_release);
}

static bool render_tvbgone_progress(std::vector<neo::srgb> &buf, int64_t now_us)
{
    if (!s_tvbgone_active.load(std::memory_order_acquire)) return false;

    uint8_t bri = BRIGHTNESS_LEVELS[s_brightness_idx.load(std::memory_order_relaxed)];
    neo::srgb purple = scale_pixel(
        neo::srgb{TVBGONE_PURPLE_R, TVBGONE_PURPLE_G, TVBGONE_PURPLE_B}, bri);

    int solid_count = s_tvbgone_solid_count.load(std::memory_order_relaxed);
    if (solid_count < 1) solid_count = 1;
    if (solid_count > static_cast<int>(NUM_LEDS)) solid_count = static_cast<int>(NUM_LEDS);

    for (auto &px : buf) px = neo::srgb{0, 0, 0};

    for (int i = 0; i < solid_count; ++i)
        buf[i] = purple;

    if (solid_count < static_cast<int>(NUM_LEDS)) {
        int64_t phase = (now_us / 1000) / TVBGONE_BLINK_HALF_PERIOD_MS;
        if ((phase % 2) == 0) buf[solid_count] = purple;
    }

    led_transmit(buf);
    return true;
}

static bool render_fire(std::vector<neo::srgb> &buf, int64_t now_us)
{
    if (!s_eff_fire.active.load(std::memory_order_acquire)) return false;
    int64_t elapsed_ms = (now_us - s_eff_fire.start_us.load()) / 1000;
    if (elapsed_ms >= FIRE_DURATION_MS) {
        s_eff_fire.active.store(false, std::memory_order_release);
        return false;
    }

    uint8_t base_bri = BRIGHTNESS_LEVELS[s_brightness_idx.load()];
    uint8_t cr = s_eff_fire.r.load();
    uint8_t cg = s_eff_fire.g.load();
    uint8_t cb = s_eff_fire.b.load();

    for (auto &px : buf) px = neo::srgb{0, 0, 0};

    /* How many of the 9 fill steps are currently lit? */
    int active_steps;

    if (elapsed_ms < FIRE_FILL_MS) {
        /* Fill phase: one new step every FIRE_FILL_MS / FIRE_NUM_STEPS ms */
        active_steps = std::min(
            static_cast<int>((elapsed_ms * FIRE_NUM_STEPS) / FIRE_FILL_MS) + 1,
            FIRE_NUM_STEPS);
    } else if (elapsed_ms < FIRE_FILL_MS + FIRE_HOLD_MS) {
        active_steps = FIRE_NUM_STEPS;
    } else {
        /* Fade phase: steps peel off from the top (crown first).
         * Step durations grow linearly so the last one is 3× the first.
         * d_i = d0 * (1 + 2i/8),  sum_{i=0..8} d_i = 18 * d0 = FIRE_FADE_MS */
        int64_t fade_elapsed = elapsed_ms - FIRE_FILL_MS - FIRE_HOLD_MS;
        constexpr float d0 = static_cast<float>(FIRE_FADE_MS) / 18.0f;
        int turned_off = 0;
        float cumulative = 0.0f;
        for (int i = 0; i < FIRE_NUM_STEPS; ++i) {
            cumulative += d0 * (1.0f + 2.0f * static_cast<float>(i) / 8.0f);
            if (static_cast<float>(fade_elapsed) >= cumulative)
                turned_off = i + 1;
            else
                break;
        }
        active_steps = FIRE_NUM_STEPS - turned_off;
    }

    /* Paint every active step */
    for (int s = 0; s < active_steps; ++s) {
        const auto &step = FIRE_STEPS[s];

        if (s == FIRE_NUM_STEPS - 1) {
            /* Crown LED: white at 4× brightness */
            uint8_t bri = static_cast<uint8_t>(
                std::min<uint16_t>(static_cast<uint16_t>(base_bri) * 4, 255));
            buf[step.indices[0]] = neo::srgb{bri, bri, bri};
        } else {
            /* Team-color LED: brightness ramps 0.5× (step 0) → 2.5× (step 7) */
            float mult = 0.5f + 2.0f * static_cast<float>(s) / 7.0f;
            uint8_t bri = static_cast<uint8_t>(
                std::min<uint16_t>(static_cast<uint16_t>(base_bri * mult), 255));
            neo::srgb px = scale_pixel(neo::srgb{cr, cg, cb}, bri);
            for (int j = 0; j < step.count; ++j)
                buf[step.indices[j]] = px;
        }
    }

    led_transmit(buf);
    return true;
}

static bool render_hit_first(std::vector<neo::srgb> &buf, int64_t now_us)
{
    if (!s_eff_hit_first.active.load(std::memory_order_acquire)) return false;
    int64_t elapsed_ms = (now_us - s_eff_hit_first.start_us.load()) / 1000;
    if (elapsed_ms >= HIT_FIRST_DURATION_MS) {
        s_eff_hit_first.active.store(false, std::memory_order_release);
        return false;
    }

    uint8_t max_bri = BRIGHTNESS_LEVELS[s_brightness_idx.load()];
    uint8_t cr = s_eff_hit_first.r.load();
    uint8_t cg = s_eff_hit_first.g.load();
    uint8_t cb = s_eff_hit_first.b.load();

    float t = static_cast<float>(elapsed_ms) / static_cast<float>(HIT_FIRST_DURATION_MS);

    constexpr float TOTAL_REVOLUTIONS = 5.0f;
    constexpr int   TRAIL_LEN         = 6;

    /* Quintic ease-in-out: crawl → rocket → gentle stop */
    float eased;
    if (t < 0.5f) {
        float t2 = t * t;
        eased = 16.0f * t2 * t2 * t;
    } else {
        float u  = -2.0f * t + 2.0f;
        float u2 = u * u;
        eased = 1.0f - (u2 * u2 * u) / 2.0f;
    }

    float head_f = std::fmod(eased * TOTAL_REVOLUTIONS * static_cast<float>(NUM_LEDS)
                             + s_eff_hit_first.rand_offset.load(std::memory_order_relaxed),
                             static_cast<float>(NUM_LEDS));
    if (head_f < 0.0f) head_f += static_cast<float>(NUM_LEDS);
    int head = static_cast<int>(head_f) % static_cast<int>(NUM_LEDS);

    /* Base brightness: 2× the user's current setting, capped at 255 */
    uint8_t base_bri = static_cast<uint8_t>(
        std::min(255, static_cast<int>(max_bri) * 2));
    neo::srgb base_px = scale_pixel(neo::srgb{cr, cg, cb}, base_bri);
    for (auto &px : buf) px = base_px;

    /* Envelope: shadow strength fades in then out so the animation
     * begins and ends as a solid team-color fill.
     * Quadratic fade-in over first 15 %, quadratic fade-out over last 15 %. */
    float envelope;
    constexpr float FADE_FRAC = 0.15f;
    if (t < FADE_FRAC) {
        float f = t / FADE_FRAC;
        envelope = f * f;
    } else if (t > 1.0f - FADE_FRAC) {
        float f = (1.0f - t) / FADE_FRAC;
        envelope = f * f;
    } else {
        envelope = 1.0f;
    }

    /* Dark comet: head dips toward black, trail recovers to base.
     * DIM values (0 = base, 1 = black) are scaled by the envelope. */
    constexpr float DIM[TRAIL_LEN] = {1.0f, 0.70f, 0.40f, 0.20f, 0.08f, 0.02f};

    for (int i = 0; i < TRAIL_LEN; ++i) {
        int idx = ((head - i) % static_cast<int>(NUM_LEDS)
                   + static_cast<int>(NUM_LEDS)) % static_cast<int>(NUM_LEDS);

        float bri_f = base_bri * (1.0f - DIM[i] * envelope);
        uint8_t bri = static_cast<uint8_t>(std::max(0.0f, bri_f));

        buf[idx] = scale_pixel(neo::srgb{cr, cg, cb}, bri);
    }

    led_transmit(buf);
    return true;
}

static bool render_blink(timed_effect_t &eff, std::vector<neo::srgb> &buf, int64_t now_us)
{
    if (!eff.active.load(std::memory_order_acquire)) return false;
    int64_t elapsed_ms = (now_us - eff.start_us.load()) / 1000;
    if (elapsed_ms >= BLINK_TOTAL_MS) {
        eff.active.store(false, std::memory_order_release);
        return false;
    }
    uint8_t max_bri = BRIGHTNESS_LEVELS[s_brightness_idx.load()];
    int phase = static_cast<int>(elapsed_ms / BLINK_PHASE_MS);
    if (phase % 2 == 0) {
        fill_solid(buf, eff.r.load(), eff.g.load(), eff.b.load(), max_bri / 2);
    } else {
        for (auto &px : buf) px = neo::srgb{0, 0, 0};
    }
    led_transmit(buf);
    return true;
}

static void populate_stagger_breath(neo::alarm const &a, std::vector<neo::srgb> &colors)
{
    const auto breath_period = std::chrono::milliseconds{STAGGER_BREATH_PERIOD_MS};
    const int64_t block = a.total_elapsed().count() / breath_period.count();
    const int phase = static_cast<int>(block % 2);

    float t = breath_period > 0ms ? a.cycle_time(breath_period) : 0.f;
    t = 1.f - 2.f * std::fabs(t - 0.5f);

    const uint8_t intensity =
        static_cast<uint8_t>(std::clamp(t * 255.f, 0.f, 255.f));

    neo::srgb c{
        s_stagger_breath_r.load(std::memory_order_relaxed),
        s_stagger_breath_g.load(std::memory_order_relaxed),
        s_stagger_breath_b.load(std::memory_order_relaxed),
    };

    std::size_t idx = 0;
    for (auto &px : colors) {
        if (static_cast<int>(idx % 2) == phase) {
            px = neo::srgb{
                static_cast<uint8_t>((uint16_t(c.r) * intensity) / 255),
                static_cast<uint8_t>((uint16_t(c.g) * intensity) / 255),
                static_cast<uint8_t>((uint16_t(c.b) * intensity) / 255),
            };
        } else {
            px = neo::srgb{0, 0, 0};
        }
        ++idx;
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Alarm callback — runs at FRAME_RATE FPS in its own FreeRTOS task
 * ═════════════════════════════════════════════════════════════════════════════ */

static void frame_callback(neo::alarm &a)
{
    static std::vector<neo::srgb> buf(NUM_LEDS, neo::srgb{0, 0, 0});
    static std::vector<neo::srgb> bg_buf(NUM_LEDS, neo::srgb{0, 0, 0});

    /* ── Handle background animation switch request ──────────────────────── */
    if (s_next_anim_requested.exchange(false)) {
        s_bg_index = (s_bg_index + 1) % NUM_BG_ANIMATIONS;
        s_bg_transition->transition_to(a, s_bg_pool[s_bg_index], 1s);
        ESP_LOGI(TAG, "Switched to background animation %d", s_bg_index);
    }

    /* ── Rebuild slot 8 with image palette if requested ─────────────────── */
    if (s_palette_update_requested.exchange(false)) {
        auto &p = s_pending_palette;
        s_bg_pool[8] = neo::wrap(neo::gradient_fx{
            std::vector<neo::gradient_entry>{
                {0.00f, neo::srgb{0, 0, 0}},
                {0.10f, neo::srgb{p.r1, p.g1, p.b1}},
                {0.20f, neo::srgb{p.r1, p.g1, p.b1}},
                {0.33f, neo::srgb{0, 0, 0}},
                {0.43f, neo::srgb{p.r2, p.g2, p.b2}},
                {0.53f, neo::srgb{p.r2, p.g2, p.b2}},
                {0.66f, neo::srgb{0, 0, 0}},
                {0.76f, neo::srgb{p.r3, p.g3, p.b3}},
                {0.86f, neo::srgb{p.r3, p.g3, p.b3}},
                {1.00f, neo::srgb{0, 0, 0}},
            },
            4s});
        s_bg_index = 8;
        s_stagger_breath_active.store(false, std::memory_order_relaxed);
        s_solid_color_active.store(false, std::memory_order_relaxed);
        s_bg_transition->transition_to(a, s_bg_pool[8], 1s);
        ESP_LOGI(TAG, "Image palette applied: (%d,%d,%d) (%d,%d,%d) (%d,%d,%d)",
                 p.r1, p.g1, p.b1, p.r2, p.g2, p.b2, p.r3, p.g3, p.b3);
    }

    /* ── Suppression: blank LEDs during radio bursts ─────────────────────── */
    if (s_suppressed.load(std::memory_order_relaxed)) {
        for (auto &px : buf) px = neo::srgb{0, 0, 0};
        led_transmit(buf);
        return;
    }

    int64_t now_us = esp_timer_get_time();

    /* ── Render the highest-priority active effect ───────────────────────── */
    /*10 */ if (render_tvbgone_progress(buf, now_us))        return;
    /* 9 */ if (render_hit_first(buf, now_us))              return;
    /* 8 */ if (render_blink(s_eff_hit_repeat, buf, now_us)) return;
    /* 7 */ if (render_blink(s_eff_hit_friendly, buf, now_us)) return;
    /* 6 */ if (render_fire(buf, now_us))                   return;

    /* 5 — Stagger breath (indefinite until stopped) */
    if (s_stagger_breath_active.load(std::memory_order_relaxed)) {
        populate_stagger_breath(a, bg_buf);
        uint8_t bri = BRIGHTNESS_LEVELS[s_brightness_idx.load(std::memory_order_relaxed)];
        for (std::size_t i = 0; i < NUM_LEDS; ++i) {
            buf[i] = scale_pixel(bg_buf[i], bri);
        }
        led_transmit(buf);
        return;
    }

    /* 5 — Solid color fill (same tier as stagger breath, mutually exclusive) */
    if (s_solid_color_active.load(std::memory_order_relaxed)) {
        uint8_t bri = BRIGHTNESS_LEVELS[s_brightness_idx.load(std::memory_order_relaxed)];
        fill_solid(buf,
                   s_solid_color_r.load(std::memory_order_relaxed),
                   s_solid_color_g.load(std::memory_order_relaxed),
                   s_solid_color_b.load(std::memory_order_relaxed),
                   bri);
        led_transmit(buf);
        return;
    }

    /* 4 — Button-rotation background animation */
    s_bg_transition->populate(a, bg_buf);

    uint8_t bri = BRIGHTNESS_LEVELS[s_brightness_idx.load(std::memory_order_relaxed)];
    for (std::size_t i = 0; i < NUM_LEDS; ++i) {
        buf[i] = scale_pixel(bg_buf[i], bri);
    }
    led_transmit(buf);
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Startup LED wipe (blocking, runs before the alarm)
 * ═════════════════════════════════════════════════════════════════════════════ */

/* Short wipe: brightness only affects output once frame_callback runs after alarm.start(). */
static constexpr unsigned STARTUP_WIPE_TOTAL_MS = 800;

static void startup_wipe()
{
    std::vector<neo::srgb> buf(NUM_LEDS, neo::srgb{0, 0, 0});
    uint8_t bri = BRIGHTNESS_LEVELS[s_brightness_idx.load()] / 2;

    /* Read player color from NVS (same namespace/key as lasertag) */
    uint8_t color_idx = OPENLASIR_COLOR_CYAN;
    {
        nvs_handle_t nvs;
        if (nvs_open("player", NVS_READONLY, &nvs) == ESP_OK) {
            nvs_get_u8(nvs, "color", &color_idx);
            nvs_close(nvs);
        }
    }
    uint8_t cr = 0, cg = 0, cb = 0;
    openlasir_get_color_rgb(color_idx, &cr, &cg, &cb);
    neo::srgb wipe_color{
        static_cast<uint8_t>((uint16_t(cr) * bri) / 255),
        static_cast<uint8_t>((uint16_t(cg) * bri) / 255),
        static_cast<uint8_t>((uint16_t(cb) * bri) / 255),
    };

    /* Pixel-by-pixel wipe in the player's team color */
    const unsigned step_ms = (STARTUP_WIPE_TOTAL_MS + NUM_LEDS - 1) / NUM_LEDS;
    for (std::size_t i = 0; i < NUM_LEDS; ++i) {
        buf[i] = wipe_color;
        led_transmit(buf);
        vTaskDelay(pdMS_TO_TICKS(step_ms > 0 ? step_ms : 1));
    }

    /* clear */
    for (auto &px : buf) px = neo::srgb{0, 0, 0};
    led_transmit(buf);
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Build the pool of background effects
 * ═════════════════════════════════════════════════════════════════════════════ */

static void build_bg_pool()
{
    /* 0 — Rainbow gradient, 5 s rotation (all LEDs on — the one exception) */
    s_bg_pool[0] = neo::wrap(neo::gradient_fx{
        {neo::srgb{255, 0, 0}, neo::srgb{255, 255, 0}, neo::srgb{0, 255, 0},
         neo::srgb{0, 255, 255}, neo::srgb{0, 0, 255}, neo::srgb{255, 0, 255},
         neo::srgb{255, 0, 0}},
        5s});

    /* 1 — Dual Comets: purple and cyan dots with tails chasing each other
     *     around the perimeter in opposite-ish feel (180° apart) */
    s_bg_pool[1] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{0, 0, 0}},
            {0.04f, neo::srgb{0, 5, 30}},
            {0.10f, neo::srgb{40, 0, 140}},
            {0.15f, neo::srgb{180, 40, 255}},
            {0.20f, neo::srgb{0, 0, 0}},
            {0.46f, neo::srgb{0, 0, 0}},
            {0.50f, neo::srgb{0, 15, 40}},
            {0.56f, neo::srgb{0, 80, 180}},
            {0.62f, neo::srgb{0, 220, 255}},
            {0.67f, neo::srgb{0, 0, 0}},
            {1.00f, neo::srgb{0, 0, 0}},
        },
        3s});

    /* 2 — Meteor: warm-white head with a long orange/red tail sweeping
     *     around the rectangle like a shooting star (~4-5 LEDs visible) */
    s_bg_pool[2] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{0, 0, 0}},
            {0.65f, neo::srgb{0, 0, 0}},
            {0.75f, neo::srgb{15, 2, 0}},
            {0.83f, neo::srgb{80, 15, 0}},
            {0.89f, neo::srgb{200, 60, 5}},
            {0.94f, neo::srgb{255, 180, 60}},
            {0.98f, neo::srgb{255, 240, 180}},
            {1.00f, neo::srgb{255, 255, 220}},
        },
        3s});

    /* 3 — Ocean Wave: a pulse of blue/cyan travels the perimeter with the
     *     rest of the badge dark; ~5-6 LEDs lit at any time */
    s_bg_pool[3] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{0, 0, 0}},
            {0.20f, neo::srgb{0, 0, 0}},
            {0.30f, neo::srgb{0, 5, 30}},
            {0.40f, neo::srgb{0, 40, 120}},
            {0.50f, neo::srgb{20, 160, 220}},
            {0.60f, neo::srgb{0, 40, 120}},
            {0.70f, neo::srgb{0, 5, 30}},
            {0.80f, neo::srgb{0, 0, 0}},
            {1.00f, neo::srgb{0, 0, 0}},
        },
        5s});

    /* 4 — Cyan comet with trailing fade, fast spin (~3-4 LEDs visible) */
    s_bg_pool[4] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{0, 0, 0}},
            {0.70f, neo::srgb{0, 0, 0}},
            {0.85f, neo::srgb{0, 20, 60}},
            {0.93f, neo::srgb{0, 120, 200}},
            {0.97f, neo::srgb{180, 240, 255}},
            {1.00f, neo::srgb{255, 255, 255}},
        },
        2s});

    /* 5 — Fireflies: two small green/yellow dots drift and morph between
     *     positions as the pulse_fx crossfades between two sparse rotating
     *     patterns at different speeds (~2-3 LEDs visible) */
    s_bg_pool[5] = neo::wrap(neo::pulse_fx{
        neo::gradient_fx{
            std::vector<neo::gradient_entry>{
                {0.00f, neo::srgb{0, 0, 0}},
                {0.06f, neo::srgb{0, 0, 0}},
                {0.12f, neo::srgb{60, 200, 30}},
                {0.18f, neo::srgb{0, 0, 0}},
                {0.53f, neo::srgb{0, 0, 0}},
                {0.59f, neo::srgb{200, 180, 40}},
                {0.65f, neo::srgb{0, 0, 0}},
                {1.00f, neo::srgb{0, 0, 0}},
            },
            5s},
        neo::gradient_fx{
            std::vector<neo::gradient_entry>{
                {0.00f, neo::srgb{0, 0, 0}},
                {0.28f, neo::srgb{0, 0, 0}},
                {0.34f, neo::srgb{100, 220, 50}},
                {0.40f, neo::srgb{0, 0, 0}},
                {0.73f, neo::srgb{0, 0, 0}},
                {0.79f, neo::srgb{180, 200, 30}},
                {0.85f, neo::srgb{0, 0, 0}},
                {1.00f, neo::srgb{0, 0, 0}},
            },
            8s},
        5s});

    /* 6 — Electric Arcs: three sparse purple dots morph into three sparse
     *     blue dots at offset positions; both patterns rotate, giving the
     *     impression of arcs jumping around the rectangle */
    s_bg_pool[6] = neo::wrap(neo::pulse_fx{
        neo::gradient_fx{
            std::vector<neo::gradient_entry>{
                {0.00f, neo::srgb{0, 0, 0}},
                {0.03f, neo::srgb{160, 0, 240}},
                {0.10f, neo::srgb{40, 0, 80}},
                {0.16f, neo::srgb{0, 0, 0}},
                {0.30f, neo::srgb{0, 0, 0}},
                {0.34f, neo::srgb{120, 0, 200}},
                {0.41f, neo::srgb{30, 0, 60}},
                {0.47f, neo::srgb{0, 0, 0}},
                {0.63f, neo::srgb{0, 0, 0}},
                {0.67f, neo::srgb{160, 0, 240}},
                {0.74f, neo::srgb{40, 0, 80}},
                {0.80f, neo::srgb{0, 0, 0}},
                {1.00f, neo::srgb{0, 0, 0}},
            },
            4s},
        neo::gradient_fx{
            std::vector<neo::gradient_entry>{
                {0.00f, neo::srgb{0, 0, 0}},
                {0.14f, neo::srgb{0, 0, 0}},
                {0.18f, neo::srgb{0, 80, 255}},
                {0.25f, neo::srgb{0, 15, 60}},
                {0.31f, neo::srgb{0, 0, 0}},
                {0.47f, neo::srgb{0, 0, 0}},
                {0.51f, neo::srgb{0, 60, 200}},
                {0.58f, neo::srgb{0, 10, 50}},
                {0.64f, neo::srgb{0, 0, 0}},
                {0.80f, neo::srgb{0, 0, 0}},
                {0.84f, neo::srgb{0, 80, 255}},
                {0.91f, neo::srgb{0, 15, 60}},
                {0.97f, neo::srgb{0, 0, 0}},
                {1.00f, neo::srgb{0, 0, 0}},
            },
            4s},
        3s});

    /* 7 — Lava Drop: narrow warm glow traveling slowly around the perimeter;
     *     bright yellow-white head fades through orange into deep red tail */
    s_bg_pool[7] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{0, 0, 0}},
            {0.50f, neo::srgb{0, 0, 0}},
            {0.62f, neo::srgb{20, 0, 0}},
            {0.72f, neo::srgb{120, 15, 0}},
            {0.82f, neo::srgb{255, 60, 0}},
            {0.90f, neo::srgb{255, 160, 20}},
            {0.96f, neo::srgb{255, 220, 80}},
            {1.00f, neo::srgb{255, 255, 160}},
        },
        5s});

    /* 8 — Ethereal: slow sweep of pink, purple, and ice-blue
     *     matched to the display image (pink/purple hair, blue eyes). */
    s_bg_pool[8] = neo::wrap(neo::gradient_fx{
        std::vector<neo::gradient_entry>{
            {0.00f, neo::srgb{  0,   0,   0}},
            {0.10f, neo::srgb{ 60,   0,  80}},  /* deep purple */
            {0.25f, neo::srgb{180,  60, 220}},  /* vivid purple */
            {0.40f, neo::srgb{255, 100, 180}},  /* pink */
            {0.55f, neo::srgb{255, 200, 230}},  /* soft blush white */
            {0.68f, neo::srgb{ 80, 180, 255}},  /* ice blue (eyes) */
            {0.80f, neo::srgb{ 20,  60, 140}},  /* deep blue */
            {0.90f, neo::srgb{ 60,   0,  80}},  /* back to purple */
            {1.00f, neo::srgb{  0,   0,   0}},
        },
        6s});

    /* Wrap everything in a transition_fx for smooth crossfades */
    s_bg_transition = std::make_shared<neo::transition_fx>();
}

/* ═════════════════════════════════════════════════════════════════════════════
 *  Public API — extern "C" implementations
 * ═════════════════════════════════════════════════════════════════════════════ */

extern "C" void led_controller_start(void)
{
    ESP_LOGI(TAG, "Initializing LED controller (GPIO %d, %d LEDs)",
             static_cast<int>(LED_GPIO), static_cast<int>(NUM_LEDS));

    /* ── Create the libneon encoder (owns the RMT TX channel) ────────────── */
    static neo::led_encoder encoder{
        neo::encoding::ws2812b,
        neo::make_rmt_config(LED_GPIO, /*dma=*/false, /*mem_symbols=*/48) // 48 needed or else it consumes two out of the two ESP32-C6 RMT channels (and we need one for IR)
    };
    s_encoder = &encoder;

    /* ── Startup wipe ────────────────────────────────────────────────────── */
    startup_wipe();

    /* ── Build background animation pool ─────────────────────────────────── */
    build_bg_pool();

    /* Start on the Ethereal animation (index 8) to match the display image */
    s_bg_index = 8;

    /* ── Create the alarm (FRAME_RATE frame loop) ─────────────────────────── */
    static neo::alarm alarm{FRAME_RATE, frame_callback};
    s_alarm = &alarm;

    alarm.start();

    /*
     * Kick off the first background animation via transition_to after the
     * alarm has started (transition_fx needs a running alarm for timing).
     * Small delay so alarm task is scheduled.
     */
    vTaskDelay(pdMS_TO_TICKS(50));
    s_bg_transition->transition_to(*s_alarm, s_bg_pool[s_bg_index], 2s);

    ESP_LOGI(TAG, "LED controller started — animation %d", s_bg_index);
}

/* ── Brightness ──────────────────────────────────────────────────────────── */

extern "C" void led_controller_increase_brightness(void)
{
    int idx = s_brightness_idx.load();
    if (idx < NUM_BRIGHTNESS_LEVELS - 1) {
        s_brightness_idx.store(idx + 1);
        ESP_LOGI(TAG, "Brightness UP -> %d", BRIGHTNESS_LEVELS[idx + 1]);
    }
}

extern "C" void led_controller_decrease_brightness(void)
{
    int idx = s_brightness_idx.load();
    if (idx > 0) {
        s_brightness_idx.store(idx - 1);
        ESP_LOGI(TAG, "Brightness DOWN -> %d", BRIGHTNESS_LEVELS[idx - 1]);
    }
}

/* ── Animation cycling ───────────────────────────────────────────────────── */

extern "C" void led_controller_next_animation(void)
{
    s_next_anim_requested.store(true);
}

extern "C" void led_controller_stagger_breath_start(uint8_t r, uint8_t g, uint8_t b)
{
    s_stagger_breath_r.store(r, std::memory_order_relaxed);
    s_stagger_breath_g.store(g, std::memory_order_relaxed);
    s_stagger_breath_b.store(b, std::memory_order_relaxed);
    s_stagger_breath_active.store(true, std::memory_order_release);
}

extern "C" void led_controller_stagger_breath_stop(void)
{
    s_stagger_breath_active.store(false, std::memory_order_release);
}

extern "C" bool led_controller_stagger_breath_is_active(void)
{
    return s_stagger_breath_active.load(std::memory_order_relaxed);
}

extern "C" void led_controller_solid_color_start(uint8_t r, uint8_t g, uint8_t b)
{
    s_solid_color_r.store(r, std::memory_order_relaxed);
    s_solid_color_g.store(g, std::memory_order_relaxed);
    s_solid_color_b.store(b, std::memory_order_relaxed);
    s_solid_color_active.store(true, std::memory_order_release);
}

extern "C" void led_controller_solid_color_stop(void)
{
    s_solid_color_active.store(false, std::memory_order_release);
}

extern "C" bool led_controller_solid_color_is_active(void)
{
    return s_solid_color_active.load(std::memory_order_relaxed);
}

extern "C" void led_controller_set_brightness_idx(int idx)
{
    int wrapped = idx % NUM_BRIGHTNESS_LEVELS;
    if (wrapped < 0) wrapped += NUM_BRIGHTNESS_LEVELS;
    s_brightness_idx.store(wrapped);
}

extern "C" void led_controller_trigger_fire(uint8_t r, uint8_t g, uint8_t b)
{
    activate_effect(s_eff_fire, r, g, b);
}

extern "C" void led_controller_trigger_hit_first(uint8_t r, uint8_t g, uint8_t b)
{
    activate_effect(s_eff_hit_first, r, g, b);
}

extern "C" void led_controller_trigger_hit_repeat(void)
{
    activate_effect(s_eff_hit_repeat, 255, 255, 255);
}

extern "C" void led_controller_trigger_hit_friendly(uint8_t r, uint8_t g, uint8_t b)
{
    activate_effect(s_eff_hit_friendly, r, g, b);
}

extern "C" void led_controller_tvbgone_progress_start(void)
{
    s_tvbgone_active.store(true, std::memory_order_release);
}

extern "C" void led_controller_tvbgone_progress_stop(void)
{
    s_tvbgone_active.store(false, std::memory_order_release);
}

extern "C" void led_controller_tvbgone_progress_update(uint16_t current, uint16_t total)
{
    if (total == 0) return;
    int count = (current * NUM_LEDS) / total;
    if (count > static_cast<int>(NUM_LEDS)) count = NUM_LEDS;
    s_tvbgone_solid_count.store(count, std::memory_order_relaxed);
}

extern "C" void led_controller_suppress(bool suppress)
{
    s_suppressed.store(suppress, std::memory_order_relaxed);
}

extern "C" void led_controller_image_palette_start(uint8_t r1, uint8_t g1, uint8_t b1,
                                                    uint8_t r2, uint8_t g2, uint8_t b2,
                                                    uint8_t r3, uint8_t g3, uint8_t b3)
{
    /* Store palette and let frame_callback rebuild slot 8 safely */
    s_pending_palette = {r1, g1, b1, r2, g2, b2, r3, g3, b3};
    s_palette_update_requested.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "Image palette queued: (%d,%d,%d) (%d,%d,%d) (%d,%d,%d)",
             r1, g1, b1, r2, g2, b2, r3, g3, b3);
}
