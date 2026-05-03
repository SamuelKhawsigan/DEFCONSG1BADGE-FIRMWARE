/*
 * libneon_led_controller.h
 *
 * Public API for the LED animation controller module.
 * Drives a WS2812B NeoPixel strip via the libneon library, running
 * background animations and accepting on-demand triggered effects
 * from other modules (e.g. lasertag).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the LED strip hardware and start the animation loop.
 * Must be called after NVS init but before any module that triggers LED effects.
 */
void led_controller_start(void);

/* ── Brightness (called from button callbacks) ───────────────────────────── */

void led_controller_increase_brightness(void);
void led_controller_decrease_brightness(void);

/* ── Background animation cycling ────────────────────────────────────────── */

void led_controller_next_animation(void);

/**
 * Alternating-LED breathing until stopped. Does not participate in button cycling.
 * Short lasertag triggers still override while they run. RGB is full brightness;
 * global brightness scaling still applies in the frame loop.
 */
void led_controller_stagger_breath_start(uint8_t r, uint8_t g, uint8_t b);
void led_controller_stagger_breath_stop(void);
bool led_controller_stagger_breath_is_active(void);

/**
 * Solid-color fill on all LEDs until stopped.  Same priority tier as
 * stagger breath — caller is responsible for only having one active.
 * Global brightness scaling still applies.
 */
void led_controller_solid_color_start(uint8_t r, uint8_t g, uint8_t b);
void led_controller_solid_color_stop(void);
bool led_controller_solid_color_is_active(void);

/* ── Direct brightness index control ─────────────────────────────────────── */

/**
 * Set the brightness index directly (wraps around BRIGHTNESS_LEVELS).
 */
void led_controller_set_brightness_idx(int idx);

/* ── Triggered effects (called by lasertag game logic) ───────────────────── */

void led_controller_trigger_fire(uint8_t r, uint8_t g, uint8_t b);
void led_controller_trigger_hit_first(uint8_t r, uint8_t g, uint8_t b);
void led_controller_trigger_hit_repeat(void);
void led_controller_trigger_hit_friendly(uint8_t r, uint8_t g, uint8_t b);

/* ── TV-B-Gone progress clock ─────────────────────────────────────────────
 *
 * Highest-priority animation: a purple "clock" that fills LEDs left to
 * right as a progress bar.  Solid LEDs show completed progress; the next
 * LED blinks at 1 Hz.  Call _update() periodically with real progress
 * from tvbgone_core_get_status().
 */
void led_controller_tvbgone_progress_start(void);
void led_controller_tvbgone_progress_stop(void);

/**
 * Update the progress indicator.
 * @param current  Current code number (1-based, from tvbgone_core_get_status).
 * @param total    Total codes for this send.
 */
void led_controller_tvbgone_progress_update(uint16_t current, uint16_t total);

/* ── Suppression (blank LEDs during ESP-NOW radio bursts) ────────────────── */

void led_controller_suppress(bool suppress);

/* ── Image palette animation ────────────────────────────────────────────── */

/**
 * @brief Rebuild animation slot 8 with 3 colors sampled from the displayed image.
 *
 * The 3 colors sweep around the badge as a gradient that cycles every 4 s.
 * Call after lcd_display_show_jpg() + lcd_display_get_palette().
 * Brightness controls still work as normal.
 */
void led_controller_image_palette_start(uint8_t r1, uint8_t g1, uint8_t b1,
                                        uint8_t r2, uint8_t g2, uint8_t b2,
                                        uint8_t r3, uint8_t g3, uint8_t b3);

#ifdef __cplusplus
}
#endif
