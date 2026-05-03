/*
 * IR encoder for OpenLASIR protocol.
 * Wire-level encoding is identical to NEC (same timings), so this file is
 * functionally equivalent to the original ir_nec_encoder.c with renamed types.
 *
 * Based on Espressif ESP-IDF NEC encoder example (Unlicense / CC0-1.0).
 */

#include "esp_check.h"
#include "ir_openlasir_encoder.h"

static const char *TAG = "openlasir_encoder";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_encoder_t *bytes_encoder;
    rmt_symbol_word_t leading_symbol;
    rmt_symbol_word_t ending_symbol;
    int state;
} rmt_ir_openlasir_encoder_t;

RMT_ENCODER_FUNC_ATTR
static size_t rmt_encode_ir_openlasir(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                      const void *primary_data, size_t data_size,
                                      rmt_encode_state_t *ret_state)
{
    rmt_ir_openlasir_encoder_t *enc = __containerof(encoder, rmt_ir_openlasir_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    ir_openlasir_scan_code_t *scan_code = (ir_openlasir_scan_code_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = enc->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = enc->bytes_encoder;

    switch (enc->state) {
    case 0: // leading code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &enc->leading_symbol, sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fall-through
    case 1: // address (16 bits: block_id + inverted block_id)
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel,
                                                 &scan_code->address, sizeof(uint16_t),
                                                 &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 2;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fall-through
    case 2: // command (16 bits: device_id + mode + data)
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel,
                                                 &scan_code->command, sizeof(uint16_t),
                                                 &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 3;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fall-through
    case 3: // ending / stop bit
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &enc->ending_symbol, sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ir_openlasir_encoder(rmt_encoder_t *encoder)
{
    rmt_ir_openlasir_encoder_t *enc = __containerof(encoder, rmt_ir_openlasir_encoder_t, base);
    rmt_del_encoder(enc->copy_encoder);
    rmt_del_encoder(enc->bytes_encoder);
    free(enc);
    return ESP_OK;
}

RMT_ENCODER_FUNC_ATTR
static esp_err_t rmt_ir_openlasir_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ir_openlasir_encoder_t *enc = __containerof(encoder, rmt_ir_openlasir_encoder_t, base);
    rmt_encoder_reset(enc->copy_encoder);
    rmt_encoder_reset(enc->bytes_encoder);
    enc->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_ir_openlasir_encoder(const ir_openlasir_encoder_config_t *config,
                                       rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_ir_openlasir_encoder_t *enc = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    enc = rmt_alloc_encoder_mem(sizeof(rmt_ir_openlasir_encoder_t));
    ESP_GOTO_ON_FALSE(enc, ESP_ERR_NO_MEM, err, TAG, "no mem for openlasir encoder");
    enc->base.encode = rmt_encode_ir_openlasir;
    enc->base.del    = rmt_del_ir_openlasir_encoder;
    enc->base.reset  = rmt_ir_openlasir_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &enc->copy_encoder),
                      err, TAG, "create copy encoder failed");

    enc->leading_symbol = (rmt_symbol_word_t) {
        .level0    = 1,
        .duration0 = 9000ULL * config->resolution / 1000000,   // 9 ms mark
        .level1    = 0,
        .duration1 = 4500ULL * config->resolution / 1000000,   // 4.5 ms space
    };
    enc->ending_symbol = (rmt_symbol_word_t) {
        .level0    = 1,
        .duration0 = 560 * config->resolution / 1000000,       // 560 us mark
        .level1    = 0,
        .duration1 = 0x7FFF,
    };

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0    = 1,
            .duration0 = 560 * config->resolution / 1000000,   // 560 us mark
            .level1    = 0,
            .duration1 = 560 * config->resolution / 1000000,   // 560 us space
        },
        .bit1 = {
            .level0    = 1,
            .duration0 = 560 * config->resolution / 1000000,   // 560 us mark
            .level1    = 0,
            .duration1 = 1690 * config->resolution / 1000000,  // 1690 us space
        },
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &enc->bytes_encoder),
                      err, TAG, "create bytes encoder failed");

    *ret_encoder = &enc->base;
    return ESP_OK;
err:
    if (enc) {
        if (enc->bytes_encoder) {
            rmt_del_encoder(enc->bytes_encoder);
        }
        if (enc->copy_encoder) {
            rmt_del_encoder(enc->copy_encoder);
        }
        free(enc);
    }
    return ret;
}
