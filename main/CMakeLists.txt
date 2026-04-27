idf_component_register(
    SRCS "main.cpp" "wifi.cpp" "console_task.cpp" "dataprocess.cpp" "spi_link.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_wifi nvs_flash esp_driver_uart esp_driver_spi esp_timer lwip console
)
