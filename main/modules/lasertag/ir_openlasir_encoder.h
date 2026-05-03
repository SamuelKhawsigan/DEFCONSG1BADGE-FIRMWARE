/*
 * IR encoder for OpenLASIR protocol (modified NEC).
 *
 * Wire-level encoding is identical to NEC: 9 ms leader mark, 4.5 ms space,
 * 32 data bits (560 us mark + variable space), and a stop bit at 38 kHz.
 * The difference is only in how the 32-bit payload is interpreted:
 *   Bytes 0-1: 8-bit Block ID + inverted copy (address error check)
 *   Bytes 2-3: 16-bit command with no error check
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OpenLASIR scan code for transmission
 *
 * address: block_id | (~block_id << 8)   — 8-bit ID with inverted error check
 * command: device_id | (mode << 8) | (data << 13)
 */
typedef struct {
    uint16_t address;
    uint16_t command;
} ir_openlasir_scan_code_t;

typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} ir_openlasir_encoder_config_t;

/**
 * @brief Build an ir_openlasir_scan_code_t from logical address (block_id) and command.
 *        Automatically adds the inverted error-check byte to the address field.
 */
static inline ir_openlasir_scan_code_t ir_openlasir_make_scan_code(uint8_t address, uint16_t command)
{
    ir_openlasir_scan_code_t sc = {
        .address = (uint16_t)address | ((uint16_t)(address ^ 0xFF) << 8),
        .command = command,
    };
    return sc;
}

/**
 * @brief Create RMT encoder for encoding an OpenLASIR frame into RMT symbols
 */
esp_err_t rmt_new_ir_openlasir_encoder(const ir_openlasir_encoder_config_t *config,
                                       rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
