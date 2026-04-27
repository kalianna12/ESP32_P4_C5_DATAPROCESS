#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console_task.h"
#include "dataprocess.h"
#include "wifi.h"

namespace {

static const char *TAG = "data_process";

}  // namespace

extern "C" void app_main() {
    wifi_init_storage();

    ESP_LOGI(TAG, "Pure ESP32-P4 data-processing framework is running");
    ESP_LOGI(TAG, "Use the serial console to initialize Wi-Fi, scan, and connect");

    console_task_start();
    data_process_start();

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
