#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lcd_display.h"

static const char *TAG = "lcd_display";

// ============================================================================
// GPIO PIN CONFIGURATION (from user reference)
// ============================================================================
#define LCD_HOST          SPI2_HOST
#define LCD_PIN_NUM_MISO  -1
#define LCD_PIN_NUM_MOSI  11
#define LCD_PIN_NUM_CLK   5
#define LCD_PIN_NUM_CS    4
#define LCD_PIN_NUM_DC    3
#define LCD_PIN_NUM_RST   -1
#define LCD_PIN_NUM_BK_LIGHT -1

// ST7789 display resolution
#define LCD_H_RES         320
#define LCD_V_RES         240
#define LCD_CMD_BITS      8
#define LCD_PARAM_BITS    8
#define LCD_PCLK_HZ       (20 * 1000 * 1000)

static esp_lcd_panel_handle_t panel_handle = NULL;

void lcd_display_init(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_CLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Invert color often needed for ST7789
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    
    // For this specific 2.4" 240x320 panel, no gap offset is needed. 
    // Setting it to 0,0 perfectly maps the GRAM and eliminates the glitch bar!
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if LCD_PIN_NUM_BK_LIGHT >= 0
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, 1);
#endif
    ESP_LOGI(TAG, "LCD initialized");
}

#include "rom/tjpgd.h"

// ----------------------------------------------------------------------------
// JPEG Functions
// ----------------------------------------------------------------------------

void lcd_display_clear(void)
{
    if (!panel_handle) return;
    
    // Allocate a buffer to clear the screen in chunks
    const int fill_rows = 16;
    uint16_t *line_buf = heap_caps_malloc(LCD_H_RES * fill_rows * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!line_buf) {
        line_buf = malloc(LCD_H_RES * fill_rows * sizeof(uint16_t));
    }
    if (!line_buf) return;

    memset(line_buf, 0, LCD_H_RES * fill_rows * sizeof(uint16_t));

    for (int y = 0; y < LCD_V_RES; y += fill_rows) {
        int rows = (y + fill_rows <= LCD_V_RES) ? fill_rows : (LCD_V_RES - y);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + rows, line_buf);
    }
    free(line_buf);
}

#define TJPGD_WORKSPACE_SIZE 3100

typedef struct {
    FILE *fp;
    esp_lcd_panel_handle_t panel_handle;
    int x_off;
    int y_off;
} tjpgd_context_t;

static UINT tjpgd_in_func(JDEC *jd, uint8_t *buff, UINT ndata) {
    tjpgd_context_t *ctx = (tjpgd_context_t *)jd->device;
    if (buff) {
        return fread(buff, 1, ndata, ctx->fp);
    } else {
        return fseek(ctx->fp, ndata, SEEK_CUR) ? 0 : ndata;
    }
}

// One MCU block is at most 16x16 pixels.
// ROM tjpgd uses JD_FORMAT=0 (RGB888): 16*16*3 = 768 bytes input.
// We convert to RGB565: 16*16*2 = 512 bytes output.
// Use a single static output buffer - draw_bitmap blocks until DMA is done,
// so it is safe to reuse it on each call.
static uint16_t mcu_out_buf[16 * 16];

// 3-bucket saturation-weighted accumulators: top/middle/bottom thirds of image.
static uint32_t s_acc_r[3] = {0};
static uint32_t s_acc_g[3] = {0};
static uint32_t s_acc_b[3] = {0};
static uint32_t s_acc_w[3] = {0};  /* total weight per bucket */

// Stored palette colors (top 3 from the last decoded image).
static uint8_t s_pal_r[3] = {180, 0,   0};
static uint8_t s_pal_g[3] = {0,   180, 0};
static uint8_t s_pal_b[3] = {180, 0, 180};

static UINT tjpgd_out_func(JDEC *jd, void *bitmap, JRECT *rect) {
    tjpgd_context_t *ctx = (tjpgd_context_t *)jd->device;
    if (!ctx->panel_handle) return 0;
    
    int x_start = ctx->x_off + rect->left;
    int y_start = ctx->y_off + rect->top;
    int x_end   = ctx->x_off + rect->right + 1;
    int y_end   = ctx->y_off + rect->bottom + 1;
    
    if (x_start >= LCD_H_RES || y_start >= LCD_V_RES) return 1;
    if (x_end > LCD_H_RES) x_end = LCD_H_RES;
    if (y_end > LCD_V_RES) y_end = LCD_V_RES;
    
    int w   = rect->right  - rect->left + 1;
    int h   = rect->bottom - rect->top  + 1;
    int len = w * h;
    
    // ROM tjpgd outputs RGB888 (JD_FORMAT=0): 3 bytes per pixel.
    uint8_t *src = (uint8_t *)bitmap;
    // Determine which third of the image this MCU belongs to (0=top, 1=mid, 2=bot).
    int bucket = (rect->top < 80) ? 0 : (rect->top < 160) ? 1 : 2;
    for (int i = 0; i < len; i++) {
        uint8_t r = src[i * 3 + 0];
        uint8_t g = src[i * 3 + 1];
        uint8_t b = src[i * 3 + 2];
        // Sample every 8th pixel, saturation-weighted to ignore grey/white/black.
        if ((i & 7) == 0) {
            uint8_t max_c = r > g ? (r > b ? r : b) : (g > b ? g : b);
            uint8_t min_c = r < g ? (r < b ? r : b) : (g < b ? g : b);
            uint32_t weight = max_c - min_c;
            if (weight > 20) {
                s_acc_r[bucket] += (r * weight);
                s_acc_g[bucket] += (g * weight);
                s_acc_b[bucket] += (b * weight);
                s_acc_w[bucket] += weight;
            }
        }
        // Pack to RGB565 and byte-swap for ST7789 SPI big-endian order.
        uint16_t v = ((uint16_t)(r & 0xF8) << 8)
                   | ((uint16_t)(g & 0xFC) << 3)
                   | ((uint16_t)(b         >> 3));
        mcu_out_buf[i] = (v >> 8) | (v << 8);
    }
    
    esp_lcd_panel_draw_bitmap(ctx->panel_handle, x_start, y_start, x_end, y_end, mcu_out_buf);
    return 1;
}

void lcd_display_show_jpg(const char *path)
{
    if (!panel_handle) {
        ESP_LOGE(TAG, "Panel not initialized");
        return;
    }

    ESP_LOGI(TAG, "Opening JPG file: %s", path);
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open file %s", path);
        return;
    }

    uint8_t *workspace = malloc(TJPGD_WORKSPACE_SIZE);
    if (!workspace) {
        ESP_LOGE(TAG, "Failed to allocate workspace for tjpgd");
        fclose(fp);
        return;
    }

    // Reset 3-bucket accumulators before each decode.
    for (int k = 0; k < 3; k++) {
        s_acc_r[k] = 0; s_acc_g[k] = 0; s_acc_b[k] = 0; s_acc_w[k] = 0;
    }

    tjpgd_context_t ctx = { .fp = fp, .panel_handle = panel_handle, .x_off = 0, .y_off = 0 };
    JDEC jd;
    
    JRESULT res = jd_prepare(&jd, tjpgd_in_func, workspace, TJPGD_WORKSPACE_SIZE, &ctx);
    if (res == JDR_OK) {
        ESP_LOGI(TAG, "JPEG Image: %dx%d", jd.width, jd.height);
        ctx.x_off = (LCD_H_RES - jd.width) / 2;
        ctx.y_off = (LCD_V_RES - jd.height) / 2;
        if(ctx.x_off < 0) ctx.x_off = 0;
        if(ctx.y_off < 0) ctx.y_off = 0;
        
        res = jd_decomp(&jd, tjpgd_out_func, 0);
        if (res == JDR_OK) {
            ESP_LOGI(TAG, "Decompression successful");
            // Compute palette: one dominant color per image third.
            for (int k = 0; k < 3; k++) {
                uint32_t w = s_acc_w[k];
                uint32_t ar = w > 0 ? s_acc_r[k] / w : 0;
                uint32_t ag = w > 0 ? s_acc_g[k] / w : 0;
                uint32_t ab = w > 0 ? s_acc_b[k] / w : 0;
                // Boost: normalise so the brightest channel hits 220.
                uint32_t mx = ar > ag ? (ar > ab ? ar : ab) : (ag > ab ? ag : ab);
                if (mx > 0) {
                    ar = (ar * 220) / mx; if (ar > 255) ar = 255;
                    ag = (ag * 220) / mx; if (ag > 255) ag = 255;
                    ab = (ab * 220) / mx; if (ab > 255) ab = 255;
                } else {
                    // Fallback colors per bucket if third had no colorful pixels.
                    const uint8_t fb[3][3] = {{180,0,180},{0,180,180},{180,180,0}};
                    ar = fb[k][0]; ag = fb[k][1]; ab = fb[k][2];
                }
                s_pal_r[k] = (uint8_t)ar;
                s_pal_g[k] = (uint8_t)ag;
                s_pal_b[k] = (uint8_t)ab;
                ESP_LOGI(TAG, "Palette[%d]: R=%d G=%d B=%d", k,
                         s_pal_r[k], s_pal_g[k], s_pal_b[k]);
            }
        } else {
            ESP_LOGE(TAG, "Decompression failed: %d", res);
        }
    } else {
        ESP_LOGE(TAG, "JPEG prepare failed: %d", res);
    }
    
    free(workspace);
    fclose(fp);
}

void lcd_display_get_dominant_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r) *r = s_pal_r[0];
    if (g) *g = s_pal_g[0];
    if (b) *b = s_pal_b[0];
}

void lcd_display_get_palette(uint8_t *r1, uint8_t *g1, uint8_t *b1,
                              uint8_t *r2, uint8_t *g2, uint8_t *b2,
                              uint8_t *r3, uint8_t *g3, uint8_t *b3)
{
    if (r1) *r1 = s_pal_r[0];
    if (g1) *g1 = s_pal_g[0];
    if (b1) *b1 = s_pal_b[0];
    if (r2) *r2 = s_pal_r[1];
    if (g2) *g2 = s_pal_g[1];
    if (b2) *b2 = s_pal_b[1];
    if (r3) *r3 = s_pal_r[2];
    if (g3) *g3 = s_pal_g[2];
    if (b3) *b3 = s_pal_b[2];
}
