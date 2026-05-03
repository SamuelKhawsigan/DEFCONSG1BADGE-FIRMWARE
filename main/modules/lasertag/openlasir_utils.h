/*
 * openlasir_utils.h
 *
 * OpenLASIR protocol utilities — pure C port of the Arduino OpenLASIR_Utils.h.
 * Packet encoding/decoding, color lookups, and mode name lookups.
 *
 * Based on work OpenLASIR repository: https://github.com/danielweidman/OpenLASIR
 * See the OpenLASIR repository for protocol documentation and the original
 * Arduino and MicroPython implementations.
 *
 * MIT License — see OpenLASIR repository for full license text.
 */

#ifndef OPENLASIR_UTILS_H
#define OPENLASIR_UTILS_H

#include <stdint.h>
#include <stdbool.h>

/* ── Colors (3-bit data field, values 0-7) ────────────────────────────────── */

#define OPENLASIR_COLOR_CYAN     0
#define OPENLASIR_COLOR_MAGENTA  1
#define OPENLASIR_COLOR_YELLOW   2
#define OPENLASIR_COLOR_GREEN    3
#define OPENLASIR_COLOR_RED      4
#define OPENLASIR_COLOR_BLUE     5
#define OPENLASIR_COLOR_ORANGE   6
#define OPENLASIR_COLOR_WHITE    7
#define OPENLASIR_NUM_COLORS     8

typedef struct {
    uint8_t r, g, b;
} openlasir_rgb_t;

static const openlasir_rgb_t OPENLASIR_COLOR_RGB[OPENLASIR_NUM_COLORS] = {
    {  0, 255, 255},  // 0 = Cyan
    {255,   0, 255},  // 1 = Magenta
    {255, 255,   0},  // 2 = Yellow
    {  0, 255,   0},  // 3 = Green
    {255,   0,   0},  // 4 = Red
    {  0,   0, 255},  // 5 = Blue
    {255, 165,   0},  // 6 = Orange
    {255, 255, 255},  // 7 = White
};

static const char *const OPENLASIR_COLOR_NAMES[OPENLASIR_NUM_COLORS] = {
    "Cyan", "Magenta", "Yellow", "Green",
    "Red",  "Blue",    "Orange", "White"
};

/* ── Modes (5-bit field, values 0-31) ─────────────────────────────────────── */

#define OPENLASIR_MODE_LASER_TAG_FIRE                           0
#define OPENLASIR_MODE_USER_PRESENCE_ANNOUNCEMENT               1
#define OPENLASIR_MODE_BASE_STATION_PRESENCE_ANNOUNCEMENT       2
#define OPENLASIR_MODE_USER_TO_USER_HANDSHAKE_INITIATION        3
#define OPENLASIR_MODE_USER_TO_USER_HANDSHAKE_RESPONSE          4
#define OPENLASIR_MODE_USER_TO_BASE_STATION_HANDSHAKE_INITIATION  5
#define OPENLASIR_MODE_USER_TO_BASE_STATION_HANDSHAKE_RESPONSE    6
#define OPENLASIR_MODE_BASE_STATION_TO_USER_HANDSHAKE_INITIATION  7
#define OPENLASIR_MODE_BASE_STATION_TO_USER_HANDSHAKE_RESPONSE    8
#define OPENLASIR_MODE_COLOR_SET_TEMPORARY                      9
#define OPENLASIR_MODE_COLOR_SET_PERMANENT                     10
#define OPENLASIR_MODE_GENERAL_INTERACT                        11
#define OPENLASIR_NUM_DEFINED_MODES                            12

static const char *const OPENLASIR_MODE_NAMES[OPENLASIR_NUM_DEFINED_MODES] = {
    "laser_tag_fire",
    "user_presence_announcement",
    "base_station_presence_announcement",
    "user_to_user_handshake_initiation",
    "user_to_user_handshake_response",
    "user_to_base_station_handshake_initiation",
    "user_to_base_station_handshake_response",
    "base_station_to_user_handshake_initiation",
    "base_station_to_user_handshake_response",
    "color_set_temporary",
    "color_set_permanent",
    "general_interact"
};

/* ── Decoded packet structure ─────────────────────────────────────────────── */

typedef struct {
    uint8_t block_id;   // Block ID  (from address byte)
    uint8_t device_id;  // Device ID (command bits 0-7)
    uint8_t mode;       // Mode      (command bits 8-12)
    uint8_t data;       // Data/color(command bits 13-15)
} openlasir_packet_t;

/* ── Encoding ─────────────────────────────────────────────────────────────── */

/**
 * Encode a general OpenLASIR packet into an 8-bit address and 16-bit command.
 * The address is the block_id; the caller (or ir_openlasir_make_scan_code)
 * is responsible for adding the inverted error-check byte for transmission.
 */
static inline void openlasir_encode_general_packet(uint8_t block_id, uint8_t device_id,
                                                   uint8_t mode, uint8_t data,
                                                   uint8_t *out_address, uint16_t *out_command)
{
    *out_address = block_id;
    *out_command  = ((uint16_t)(data & 0x07) << 13)
                  | ((uint16_t)(mode & 0x1F) << 8)
                  | device_id;
}

/**
 * Encode a laser_tag_fire packet.
 * Shorthand for encode_general_packet with mode=0.
 */
static inline void openlasir_encode_laser_tag_fire(uint8_t block_id, uint8_t device_id,
                                                   uint8_t color,
                                                   uint8_t *out_address, uint16_t *out_command)
{
    openlasir_encode_general_packet(block_id, device_id,
                                    OPENLASIR_MODE_LASER_TAG_FIRE, color,
                                    out_address, out_command);
}

/* ── Decoding ─────────────────────────────────────────────────────────────── */

/**
 * Decode a general OpenLASIR packet from an 8-bit address and 16-bit command.
 */
static inline openlasir_packet_t openlasir_decode_general_packet(uint8_t address, uint16_t command)
{
    openlasir_packet_t pkt;
    pkt.block_id  = address;
    pkt.device_id = command & 0xFF;
    pkt.mode      = (command >> 8) & 0x1F;
    pkt.data      = (command >> 13) & 0x07;
    return pkt;
}

/**
 * Decode a laser_tag_fire packet. Returns true if the mode is laser_tag_fire.
 */
static inline bool openlasir_decode_laser_tag_fire(uint8_t address, uint16_t command,
                                                   openlasir_packet_t *out_pkt)
{
    *out_pkt = openlasir_decode_general_packet(address, command);
    return (out_pkt->mode == OPENLASIR_MODE_LASER_TAG_FIRE);
}

/* ── Name / RGB lookup helpers ────────────────────────────────────────────── */

static inline const char *openlasir_get_mode_name(uint8_t mode)
{
    if (mode < OPENLASIR_NUM_DEFINED_MODES) {
        return OPENLASIR_MODE_NAMES[mode];
    }
    return "unknown";
}

static inline const char *openlasir_get_color_name(uint8_t color)
{
    if (color < OPENLASIR_NUM_COLORS) {
        return OPENLASIR_COLOR_NAMES[color];
    }
    return "unknown";
}

static inline bool openlasir_get_color_rgb(uint8_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (color >= OPENLASIR_NUM_COLORS) {
        return false;
    }
    *r = OPENLASIR_COLOR_RGB[color].r;
    *g = OPENLASIR_COLOR_RGB[color].g;
    *b = OPENLASIR_COLOR_RGB[color].b;
    return true;
}

#endif /* OPENLASIR_UTILS_H */
