#include "spi_link.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "spi_protocol.h"

namespace {

constexpr char kTag[] = "spi_link";

// Host P4 (WTDKP4C5-S1 J2) <-> Display P4X-Function-EV-Board (J1)
// MOSI: GPIO21 <-> GPIO21
// MISO: GPIO22 <-> GPIO22
// SCLK: GPIO23 <-> GPIO23
// CS  : GPIO3  <-> GPIO3
// GND : must be connected on both boards
constexpr spi_host_device_t kSpiHost = SPI2_HOST;
constexpr gpio_num_t kSpiMosi = GPIO_NUM_21;
constexpr gpio_num_t kSpiMiso = GPIO_NUM_22;
constexpr gpio_num_t kSpiSclk = GPIO_NUM_23;
constexpr gpio_num_t kSpiCs = GPIO_NUM_3;
constexpr int kSpiClockHz = 1 * 1000 * 1000;

spi_device_handle_t g_spi = nullptr;

bool frequency_to_index(uint16_t detected_hz, uint8_t *detected_index) {
    if (detected_index == nullptr) {
        return false;
    }

    switch (detected_hz) {
    case 8:
        *detected_index = 0;
        return true;
    case 10:
        *detected_index = 1;
        return true;
    case 12:
        *detected_index = 2;
        return true;
    case 14:
        *detected_index = 3;
        return true;
    default:
        return false;
    }
}

}  // namespace

bool spi_link_init() {
    if (g_spi != nullptr) {
        return true;
    }

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = kSpiMosi;
    bus_cfg.miso_io_num = kSpiMiso;
    bus_cfg.sclk_io_num = kSpiSclk;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = static_cast<int>(sizeof(SpiResultPacket));

    esp_err_t err = spi_bus_initialize(kSpiHost, &bus_cfg, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = kSpiClockHz;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = kSpiCs;
    dev_cfg.queue_size = 1;

    err = spi_bus_add_device(kSpiHost, &dev_cfg, &g_spi);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(kTag, "SPI result link initialized");
    return true;
}

bool spi_link_send_result(const DetectionResult &result) {
    if (g_spi == nullptr && !spi_link_init()) {
        return false;
    }

    SpiResultPacket packet = {};
    packet.flags = result.valid ? 0x01 : 0x00;
    packet.detected_index = result.detected_index;
    packet.detected_hz = result.detected_hz;
    packet.binary_seq = result.last_binary_seq;
    packet.sample_counter = result.sample_counter;
    packet.timestamp_ms = result.timestamp_ms;
    packet.confidence = result.confidence;
    std::memcpy(packet.correlations, result.correlations.data(), sizeof(packet.correlations));

    spi_transaction_t trans = {};
    trans.length = sizeof(packet) * 8;
    trans.tx_buffer = &packet;

    const esp_err_t err = spi_device_transmit(g_spi, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "spi_device_transmit failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool spi_link_send_test_frequency(uint16_t detected_hz) {
    uint8_t detected_index = 0xFF;
    if (!frequency_to_index(detected_hz, &detected_index)) {
        ESP_LOGW(kTag, "Unsupported test frequency: %u Hz", detected_hz);
        return false;
    }

    DetectionResult result = {};
    result.valid = true;
    result.detected_index = detected_index;
    result.detected_hz = detected_hz;
    result.confidence = 1.0f;
    result.timestamp_ms = 0;
    result.correlations[detected_index] = 1.0f;

    const bool ok = spi_link_send_result(result);
    if (ok) {
        ESP_LOGI(kTag, "Sent SPI test packet: %u Hz idx=%u", detected_hz, detected_index);
    }
    return ok;
}
