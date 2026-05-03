/*
 * creator_detector.h
 *
 * Public API for the creator-detector module.
 * Passively scans for BLE advertisements from creator devices
 * (message forwarders) and reports detections.
 */
#pragma once

#include <stdint.h>

/**
 * Start the creator-detector BLE scanner.
 * Initializes NimBLE in Observer mode and begins duty-cycled passive scanning.
 *
 * Prerequisites: NVS flash must be initialized before calling this function.
 */
void creator_detector_start(void);

/**
 * Inject a synthetic CR packet into the creator-detector packet map.
 * Uses a fixed fake MAC address so repeated calls update the same slot.
 * The injected packet behaves as: CR{r},{g},{b}|2||6|{persistence_ds}
 * (solid color, priority 6). Persistence is in deciseconds (tenths of a second);
 * e.g. 10 = 1 s, 9000 = 15 minutes.
 *
 * Safe to call from any task.
 */
void creator_detector_inject_packet(uint8_t r, uint8_t g, uint8_t b, int persistence_ds);
