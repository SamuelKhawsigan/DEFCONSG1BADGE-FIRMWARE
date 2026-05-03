/*
 * tvbgone_badge.h
 *
 * Badge-level wrapper around the tv-b-gone component.
 * Shares the lasertag module's IR TX RMT channel (borrowed mode) and
 * coordinates IR RX suspension / fire suppression for the duration of
 * a TV-B-Gone sweep.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * One-time initialization.  Must be called after lasertag_start() so
 * the shared IR TX channel already exists.
 */
void tvbgone_badge_init(void);

/**
 * Start a TV-B-Gone sweep (both NA + EU regions, single pass).
 * Runs asynchronously in a dedicated task.  Automatically suspends
 * lasertag IR TX/RX for the duration.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already running.
 */
esp_err_t tvbgone_badge_start(void);

/**
 * Request the running TV-B-Gone sweep to stop.  The actual teardown is
 * asynchronous; the sweep task finishes shortly after this call.
 * No-op if not currently running.
 */
void tvbgone_badge_stop(void);

/**
 * @return true while a TV-B-Gone sweep is in progress.
 */
bool tvbgone_badge_is_running(void);
