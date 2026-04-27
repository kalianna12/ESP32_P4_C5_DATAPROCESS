#include "spi_link.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

namespace {

constexpr char kTag[] = "spi_link";

// Adjust these GPIOs to match the real P4-to-P4 wiring.
constexpr spi_host_device_t kSpiHost = SPI2_HOST;
constexpr gpio_num_t kSpiMosi = GPIO_NUM_11;
constexpr gpio_num_t kSpiMiso = GPIO_NUM_NC;
constexpr gpio_num_t kSpiSclk = GPIO_NUM_12;
constexpr gpio_num_t kSpiCs = GPIO_NUM_10;
constexpr int kSpiClockHz = 1 * 1000 * 1000;

constexpr uint16_t kPacketMagic = 0x5353;
constexpr uint8_t kPacketVersion = 1;

struct __attribute__((packed)) SpiResultPacket {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint8_t detected_index;
    uint8_t reserved0;
    uint16_t detected_hz;
    uint16_t binary_seq;
    uint16_t reserved1;
    uint32_t sample_counter;
    uint32_t timestamp_ms;
    float confidence;
    float correlations[4];
};

spi_device_handle_t g_spi = nullptr;

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
    packet.magic = kPacketMagic;
    packet.version = kPacketVersion;
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
