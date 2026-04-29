#pragma once

#include <cstdint>

constexpr uint16_t kSpiResultPacketMagic = 0x5353;
constexpr uint8_t kSpiResultPacketVersion = 1;

// Detection index matches the display-side ssvep_freq_t enum:
// 0=8Hz, 1=10Hz, 2=12Hz, 3=14Hz.
struct __attribute__((packed)) SpiResultPacket {
    uint16_t magic = kSpiResultPacketMagic;
    uint8_t version = kSpiResultPacketVersion;
    uint8_t flags = 0;
    uint8_t detected_index = 0xFF;
    uint8_t reserved0 = 0;
    uint16_t detected_hz = 0;
    uint16_t binary_seq = 0;
    uint16_t reserved1 = 0;
    uint32_t sample_counter = 0;
    uint32_t timestamp_ms = 0;
    float confidence = 0.0f;
    float correlations[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};
