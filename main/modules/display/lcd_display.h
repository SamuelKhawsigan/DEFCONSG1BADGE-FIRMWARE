#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SPI bus and the ST7789 LCD display.
 */
void lcd_display_init(void);

/**
 * @brief Decode and display a JPG image from a file on the screen.
 * 
 * @param path Full VFS path to the JPG file (e.g. "/spiffs/image.jpg")
 */
void lcd_display_show_jpg(const char *path);

/**
 * @brief Clear the display with black color.
 */
void lcd_display_clear(void);

/**
 * @brief Show the diagnostic test pattern.
 */
void lcd_display_test_pattern(void);

/**
 * @brief Get the dominant (average) color sampled from the last decoded JPEG.
 *
 * Call this after lcd_display_show_jpg() to retrieve the dominant RGB color,
 * which can be used to drive LED animations to match the displayed image.
 *
 * @param r  Output: red channel   (0-255)
 * @param g  Output: green channel (0-255)
 * @param b  Output: blue channel  (0-255)
 */
void lcd_display_get_dominant_color(uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief Get the 3-color palette sampled from the last decoded JPEG.
 *
 * The image is divided into top/middle/bottom thirds; the dominant vibrant
 * color from each third is returned. Call after lcd_display_show_jpg().
 */
void lcd_display_get_palette(uint8_t *r1, uint8_t *g1, uint8_t *b1,
                              uint8_t *r2, uint8_t *g2, uint8_t *b2,
                              uint8_t *r3, uint8_t *g3, uint8_t *b3);

#ifdef __cplusplus
}
#endif
