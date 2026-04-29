#pragma once

#include <array>
#include <cstdint>

    struct DetectionResult {
    bool valid = false;
    uint8_t detected_index = 0xFF;
    uint16_t detected_hz = 0;
    float confidence = 0.0f;
    std::array<float, 4> correlations = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t sample_counter = 0;
    uint16_t last_binary_seq = 0;
    uint32_t timestamp_ms = 0;
};

enum class DataStreamFormat : uint8_t {
    Unknown = 0,
    BinaryAds1299 = 1,
    TextDelimited = 2,
};

struct DataProcessStatus {
    bool task_started = false;
    bool tcp_listening = false;
    bool client_connected = false;
    DataStreamFormat stream_format = DataStreamFormat::Unknown;
    char client_ip[16] = {0};
    uint16_t client_port = 0;
    uint32_t samples_received = 0;
    uint32_t last_rx_timestamp_ms = 0;
    uint32_t detection_count = 0;
    uint32_t crc_error_count = 0;
    uint32_t text_parse_error_count = 0;
    bool has_binary_seq = false;
    uint16_t last_binary_seq = 0;
    DetectionResult last_detection = {};
};

struct DataProcessConfigSnapshot {
    std::array<bool, 8> channel_enabled = {true, true, true, true, true, true, true, true};
    int sample_rate_hz = 250;
    std::array<uint16_t, 4> target_freq_hz = {8, 10, 12, 14};
    bool filter_enabled = false;
    bool high_pass_enabled = false;
    float high_pass_cutoff_hz = 1.0f;
    bool notch_enabled = false;
    float notch_hz = 50.0f;
    float notch_q = 30.0f;
};

void data_process_start();
bool data_process_get_status(DataProcessStatus *out_status);
bool data_process_get_config(DataProcessConfigSnapshot *out_config);
bool data_process_set_sample_rate(int sample_rate_hz);
bool data_process_set_filter_enabled(bool enabled);
bool data_process_set_high_pass(bool enabled, float cutoff_hz);
bool data_process_set_notch(bool enabled, float notch_hz, float notch_q);
bool data_process_set_channels_from_list(const char *channel_list);
void data_process_reset_runtime();
