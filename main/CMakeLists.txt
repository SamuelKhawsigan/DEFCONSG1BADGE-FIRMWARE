idf_component_register(SRCS "main.c" "ta_parser.c"
                             "modules/lasertag/lasertag_main.c"
                             "modules/lasertag/ir_openlasir_encoder.c"
                             "modules/lasertag/sync_string_obf.c"
                             "modules/tvbgone/tvbgone_badge.c"
                             "modules/creator_detector/creator_detector.c"
                             "modules/libneon_led_controller/libneon_led_controller.cpp"
                             "modules/wifi_manager/wifi_manager.c"
                             "modules/ota_updater/ota_updater.c"
                             "modules/display/lcd_display.c"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio esp_timer
                                     esp_driver_gptimer esp_lcd esp_jpeg
                                     nvs_flash esp_wifi esp_netif esp_event
                                     led_strip button frotz spiffs libneon
                                     esp_driver_usb_serial_jtag vfs bt
                                     tv-b-gone mbedtls app_update esp_http_client
                                     esp_https_ota
                       INCLUDE_DIRS "." "modules/lasertag" "modules/creator_detector"
                                    "modules/libneon_led_controller" "modules/wifi_manager"
                                    "modules/tvbgone" "modules/ota_updater" "modules/display")
