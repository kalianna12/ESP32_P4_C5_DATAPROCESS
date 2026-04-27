#include "dataprocess.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

#include "spi_link.h"
#include "wifi.h"

namespace {

constexpr char kTag[] = "dataprocess";
constexpr float kPi = 3.14159265358979323846f;

constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;
constexpr int kTcpPort = 1234;
constexpr int kChannels = 8;
constexpr int kDefaultSampleRateHz = 250;
constexpr int kWindowSamples = 250;
constexpr int kDetectionStrideSamples = 62;
constexpr int kReferenceHarmonics = 2;
constexpr float kDetectionThreshold = 0.35f;
constexpr size_t kRxChunkSize = 1024;
constexpr size_t kBinaryFrameSize = 37;
constexpr int kAcceptPollDelayMs = 250;
constexpr int kRecvTimeoutMs = 200;
constexpr int kReconnectDelayMs = 300;
constexpr uint32_t kFormatProbeBytes = 64;

enum class StreamFormat : uint8_t {
    Unknown = 0,
    BinaryAds1299,
    TextDelimited,
};

static DataStreamFormat to_public_stream_format(StreamFormat format) {
    switch (format) {
    case StreamFormat::BinaryAds1299:
        return DataStreamFormat::BinaryAds1299;
    case StreamFormat::TextDelimited:
        return DataStreamFormat::TextDelimited;
    case StreamFormat::Unknown:
    default:
        return DataStreamFormat::Unknown;
    }
}

struct HighPassState {
    float previous_input = 0.0f;
    float previous_output = 0.0f;
};

struct BiquadState {
    float b0 = 1.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float x1 = 0.0f;
    float x2 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;
};

struct FilterConfig {
    bool enabled = false;
    bool high_pass_enabled = false;
    float high_pass_cutoff_hz = 1.0f;
    bool notch_enabled = false;
    float notch_hz = 50.0f;
    float notch_q = 30.0f;
};

struct DetectorConfig {
    std::array<bool, kChannels> channel_enabled = {true, true, true, true, true, true, true, true};
    int sample_rate_hz = kDefaultSampleRateHz;
    std::array<uint16_t, 4> target_freq_hz = {8, 10, 12, 14};
    FilterConfig filter = {};
};

struct ConnectionState {
    StreamFormat format = StreamFormat::Unknown;
    std::vector<uint8_t> binary_buffer;
    std::string text_buffer;
    std::vector<uint8_t> probe_buffer;
};

class DataProcessor {
public:
    void run();
    void get_status(DataProcessStatus &out_status) const;
    void get_config(DataProcessConfigSnapshot &out_config) const;
    bool set_sample_rate(int sample_rate_hz);
    bool set_filter_enabled(bool enabled);
    bool set_high_pass(bool enabled, float cutoff_hz);
    bool set_notch(bool enabled, float notch_hz, float notch_q);
    bool set_channels_from_list(const char *channel_list);
    void external_reset_runtime();

private:
    void reset_runtime_state();
    void try_autoconnect_wifi();
    void serve_once();
    void handle_client(int client_sock, const sockaddr_in &client_addr);
    void feed_bytes(const uint8_t *data, size_t len);
    void detect_stream_format();
    void process_binary_buffer();
    void process_text_buffer();
    void process_binary_frame(const uint8_t *frame);
    void process_text_line(const std::string &line);
    bool parse_text_channels(const std::string &line, std::array<float, kChannels> &values) const;
    void process_sample(const std::array<float, kChannels> &values, bool has_seq, uint16_t seq);
    float apply_filters(int channel, float sample);
    void init_filter_states();
    void update_notch_coefficients();
    bool should_run_detection() const;
    DetectionResult compute_cca();
    int selected_channel_count() const;

    static uint8_t calc_crc(const uint8_t *data, size_t len);
    static bool is_ascii_like(uint8_t byte);
    static std::string trim_copy(const std::string &input);
    static bool parse_float_str(const std::string &text, float &out_value);
    static bool split_csv_channels(const std::string &line, std::array<float, kChannels> &values);
    static float dominant_eigenvalue_power_iteration(const std::vector<float> &matrix, int n);
    static bool invert_matrix(const std::vector<float> &input, std::vector<float> &output, int n);
    static std::vector<float> multiply_matrix(const std::vector<float> &a, const std::vector<float> &b, int rows, int inner, int cols);
    static std::vector<float> transpose_matrix(const std::vector<float> &matrix, int rows, int cols);

    DetectorConfig config_;
    std::array<std::array<float, kWindowSamples>, kChannels> ring_{};
    std::array<HighPassState, kChannels> high_pass_{};
    std::array<BiquadState, kChannels> notch_{};
    ConnectionState connection_;
    uint32_t samples_received_ = 0;
    uint32_t last_detection_sample_ = 0;
    uint16_t last_binary_seq_ = 0;
    bool has_binary_seq_ = false;
    bool tcp_listening_ = false;
    bool client_connected_ = false;
    uint32_t last_rx_timestamp_ms_ = 0;
    uint32_t detection_count_ = 0;
    uint32_t crc_error_count_ = 0;
    uint32_t text_parse_error_count_ = 0;
    DetectionResult last_detection_result_ = {};
    char client_ip_[16] = {0};
    uint16_t client_port_ = 0;
};

portMUX_TYPE g_data_lock = portMUX_INITIALIZER_UNLOCKED;

uint8_t DataProcessor::calc_crc(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
    }
    return crc;
}

bool DataProcessor::is_ascii_like(uint8_t byte) {
    return byte == '\n' || byte == '\r' || byte == '\t' || (byte >= 0x20 && byte <= 0x7E);
}

std::string DataProcessor::trim_copy(const std::string &input) {
    size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
        ++begin;
    }

    size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }

    return input.substr(begin, end - begin);
}

bool DataProcessor::parse_float_str(const std::string &text, float &out_value) {
    char *end = nullptr;
    errno = 0;
    const float value = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || errno == ERANGE) {
        return false;
    }
    out_value = value;
    return true;
}

bool DataProcessor::split_csv_channels(const std::string &line, std::array<float, kChannels> &values) {
    std::vector<std::string> fields;
    size_t start = 0;
    while (start <= line.size()) {
        const size_t comma = line.find(',', start);
        const size_t end = (comma == std::string::npos) ? line.size() : comma;
        fields.push_back(trim_copy(line.substr(start, end - start)));
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }

    if (fields.size() >= kChannels) {
        bool first_block_ok = true;
        for (int i = 0; i < kChannels; ++i) {
            if (!parse_float_str(fields[i], values[i])) {
                first_block_ok = false;
                break;
            }
        }
        if (first_block_ok) {
            return true;
        }
    }

    if (fields.size() >= static_cast<size_t>(kChannels + 3)) {
        bool channel_block_ok = true;
        for (int i = 0; i < kChannels; ++i) {
            if (!parse_float_str(fields[i + 3], values[i])) {
                channel_block_ok = false;
                break;
            }
        }
        if (channel_block_ok) {
            return true;
        }
    }

    return false;
}

std::vector<float> DataProcessor::transpose_matrix(const std::vector<float> &matrix, int rows, int cols) {
    std::vector<float> out(static_cast<size_t>(rows * cols), 0.0f);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            out[static_cast<size_t>(c * rows + r)] = matrix[static_cast<size_t>(r * cols + c)];
        }
    }
    return out;
}

std::vector<float> DataProcessor::multiply_matrix(const std::vector<float> &a, const std::vector<float> &b, int rows, int inner, int cols) {
    std::vector<float> out(static_cast<size_t>(rows * cols), 0.0f);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < inner; ++k) {
                sum += a[static_cast<size_t>(r * inner + k)] * b[static_cast<size_t>(k * cols + c)];
            }
            out[static_cast<size_t>(r * cols + c)] = sum;
        }
    }
    return out;
}

bool DataProcessor::invert_matrix(const std::vector<float> &input, std::vector<float> &output, int n) {
    output.assign(static_cast<size_t>(n * n), 0.0f);
    std::vector<float> work = input;

    for (int i = 0; i < n; ++i) {
        output[static_cast<size_t>(i * n + i)] = 1.0f;
    }

    for (int pivot = 0; pivot < n; ++pivot) {
        int best_row = pivot;
        float best_abs = std::fabs(work[static_cast<size_t>(pivot * n + pivot)]);
        for (int r = pivot + 1; r < n; ++r) {
            const float candidate = std::fabs(work[static_cast<size_t>(r * n + pivot)]);
            if (candidate > best_abs) {
                best_abs = candidate;
                best_row = r;
            }
        }

        if (best_abs < 1e-6f) {
            return false;
        }

        if (best_row != pivot) {
            for (int c = 0; c < n; ++c) {
                std::swap(work[static_cast<size_t>(pivot * n + c)], work[static_cast<size_t>(best_row * n + c)]);
                std::swap(output[static_cast<size_t>(pivot * n + c)], output[static_cast<size_t>(best_row * n + c)]);
            }
        }

        const float diag = work[static_cast<size_t>(pivot * n + pivot)];
        for (int c = 0; c < n; ++c) {
            work[static_cast<size_t>(pivot * n + c)] /= diag;
            output[static_cast<size_t>(pivot * n + c)] /= diag;
        }

        for (int r = 0; r < n; ++r) {
            if (r == pivot) {
                continue;
            }
            const float factor = work[static_cast<size_t>(r * n + pivot)];
            if (std::fabs(factor) < 1e-9f) {
                continue;
            }
            for (int c = 0; c < n; ++c) {
                work[static_cast<size_t>(r * n + c)] -= factor * work[static_cast<size_t>(pivot * n + c)];
                output[static_cast<size_t>(r * n + c)] -= factor * output[static_cast<size_t>(pivot * n + c)];
            }
        }
    }

    return true;
}

float DataProcessor::dominant_eigenvalue_power_iteration(const std::vector<float> &matrix, int n) {
    std::vector<float> v(static_cast<size_t>(n), 1.0f / static_cast<float>(n));
    std::vector<float> w(static_cast<size_t>(n), 0.0f);

    for (int iter = 0; iter < 16; ++iter) {
        std::fill(w.begin(), w.end(), 0.0f);
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                w[static_cast<size_t>(r)] += matrix[static_cast<size_t>(r * n + c)] * v[static_cast<size_t>(c)];
            }
        }

        float norm = 0.0f;
        for (float value : w) {
            norm += value * value;
        }
        norm = std::sqrt(std::max(norm, 1e-12f));
        for (int i = 0; i < n; ++i) {
            v[static_cast<size_t>(i)] = w[static_cast<size_t>(i)] / norm;
        }
    }

    std::fill(w.begin(), w.end(), 0.0f);
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            w[static_cast<size_t>(r)] += matrix[static_cast<size_t>(r * n + c)] * v[static_cast<size_t>(c)];
        }
    }

    float num = 0.0f;
    float den = 0.0f;
    for (int i = 0; i < n; ++i) {
        num += v[static_cast<size_t>(i)] * w[static_cast<size_t>(i)];
        den += v[static_cast<size_t>(i)] * v[static_cast<size_t>(i)];
    }
    return (den > 0.0f) ? (num / den) : 0.0f;
}

void DataProcessor::init_filter_states() {
    for (auto &state : high_pass_) {
        state = {};
    }
    for (auto &state : notch_) {
        state = {};
    }
    update_notch_coefficients();
}

void DataProcessor::update_notch_coefficients() {
    const float sample_rate = static_cast<float>(config_.sample_rate_hz);
    const float omega = 2.0f * kPi * config_.filter.notch_hz / sample_rate;
    const float alpha = std::sin(omega) / (2.0f * std::max(config_.filter.notch_q, 0.01f));
    const float cos_omega = std::cos(omega);
    const float a0 = 1.0f + alpha;

    for (auto &state : notch_) {
        state.b0 = 1.0f / a0;
        state.b1 = -2.0f * cos_omega / a0;
        state.b2 = 1.0f / a0;
        state.a1 = -2.0f * cos_omega / a0;
        state.a2 = (1.0f - alpha) / a0;
        state.x1 = 0.0f;
        state.x2 = 0.0f;
        state.y1 = 0.0f;
        state.y2 = 0.0f;
    }
}

void DataProcessor::reset_runtime_state() {
    for (auto &channel : ring_) {
        channel.fill(0.0f);
    }
    connection_ = {};
    samples_received_ = 0;
    last_detection_sample_ = 0;
    has_binary_seq_ = false;
    last_binary_seq_ = 0;
    tcp_listening_ = false;
    client_connected_ = false;
    last_rx_timestamp_ms_ = 0;
    detection_count_ = 0;
    crc_error_count_ = 0;
    text_parse_error_count_ = 0;
    last_detection_result_ = {};
    client_ip_[0] = '\0';
    client_port_ = 0;
    init_filter_states();
}

void DataProcessor::external_reset_runtime() {
    taskENTER_CRITICAL(&g_data_lock);
    reset_runtime_state();
    taskEXIT_CRITICAL(&g_data_lock);
}

void DataProcessor::try_autoconnect_wifi() {
    if (!wifi_ensure_stack_ready()) {
        ESP_LOGW(kTag, "Hosted Wi-Fi stack is not ready yet");
        return;
    }

    if (!wifi_is_connected_with_ip()) {
        wifi_autoconnect();
    }
}

void DataProcessor::run() {
    taskENTER_CRITICAL(&g_data_lock);
    reset_runtime_state();
    taskEXIT_CRITICAL(&g_data_lock);
    try_autoconnect_wifi();

    if (!spi_link_init()) {
        ESP_LOGW(kTag, "SPI link init failed, detection will continue without display transport");
    }

    while (true) {
        if (!wifi_is_connected_with_ip()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        serve_once();
        vTaskDelay(pdMS_TO_TICKS(kReconnectDelayMs));
    }
}

void DataProcessor::serve_once() {
    const int listen_sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(kTag, "socket create failed: errno=%d", errno);
        vTaskDelay(pdMS_TO_TICKS(500));
        return;
    }

    const int one = 1;
    ::setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kTcpPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(listen_sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        ESP_LOGE(kTag, "bind failed: errno=%d", errno);
        ::close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(500));
        return;
    }

    if (::listen(listen_sock, 1) != 0) {
        ESP_LOGE(kTag, "listen failed: errno=%d", errno);
        ::close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(500));
        return;
    }

    timeval timeout = {};
    timeout.tv_sec = 0;
    timeout.tv_usec = kAcceptPollDelayMs * 1000;
    ::setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(kTag, "TCP server listening on port %d", kTcpPort);
    taskENTER_CRITICAL(&g_data_lock);
    tcp_listening_ = true;
    taskEXIT_CRITICAL(&g_data_lock);

    while (wifi_is_connected_with_ip()) {
        sockaddr_in client_addr = {};
        socklen_t client_len = sizeof(client_addr);
        const int client_sock = ::accept(listen_sock, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
        if (client_sock < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                continue;
            }
            ESP_LOGW(kTag, "accept failed: errno=%d", errno);
            break;
        }

        handle_client(client_sock, client_addr);
    }

    ::close(listen_sock);
    taskENTER_CRITICAL(&g_data_lock);
    tcp_listening_ = false;
    taskEXIT_CRITICAL(&g_data_lock);
}

void DataProcessor::handle_client(int client_sock, const sockaddr_in &client_addr) {
    taskENTER_CRITICAL(&g_data_lock);
    reset_runtime_state();
    taskEXIT_CRITICAL(&g_data_lock);

    char ipbuf[16] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));
    ESP_LOGI(kTag, "Client connected: %s:%u", ipbuf, ntohs(client_addr.sin_port));
    taskENTER_CRITICAL(&g_data_lock);
    client_connected_ = true;
    std::snprintf(client_ip_, sizeof(client_ip_), "%s", ipbuf);
    client_port_ = ntohs(client_addr.sin_port);
    taskEXIT_CRITICAL(&g_data_lock);

    timeval timeout = {};
    timeout.tv_sec = 0;
    timeout.tv_usec = kRecvTimeoutMs * 1000;
    ::setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    const int one = 1;
    ::setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    std::array<uint8_t, kRxChunkSize> rx{};
    while (wifi_is_connected_with_ip()) {
        const int len = ::recv(client_sock, rx.data(), static_cast<int>(rx.size()), 0);
        if (len == 0) {
            ESP_LOGI(kTag, "Client disconnected");
            break;
        }
        if (len < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                continue;
            }
            ESP_LOGW(kTag, "recv failed: errno=%d", errno);
            break;
        }

        feed_bytes(rx.data(), static_cast<size_t>(len));
    }

    ::close(client_sock);
    taskENTER_CRITICAL(&g_data_lock);
    client_connected_ = false;
    taskEXIT_CRITICAL(&g_data_lock);
}

void DataProcessor::feed_bytes(const uint8_t *data, size_t len) {
    if (connection_.format == StreamFormat::Unknown) {
        const size_t copy_len = std::min(len, static_cast<size_t>(kFormatProbeBytes - connection_.probe_buffer.size()));
        connection_.probe_buffer.insert(connection_.probe_buffer.end(), data, data + copy_len);
        detect_stream_format();
    }

    if (connection_.format == StreamFormat::BinaryAds1299) {
        connection_.binary_buffer.insert(connection_.binary_buffer.end(), data, data + len);
        process_binary_buffer();
        return;
    }

    connection_.text_buffer.append(reinterpret_cast<const char *>(data), len);
    process_text_buffer();
}

void DataProcessor::detect_stream_format() {
    if (connection_.probe_buffer.size() < 2) {
        return;
    }

    if (connection_.probe_buffer[0] == kHeader0 && connection_.probe_buffer[1] == kHeader1) {
        connection_.format = StreamFormat::BinaryAds1299;
        ESP_LOGI(kTag, "Detected binary ADS1299 frame stream");
        return;
    }

    bool ascii_only = true;
    for (uint8_t byte : connection_.probe_buffer) {
        if (!is_ascii_like(byte)) {
            ascii_only = false;
            break;
        }
    }

    if (ascii_only && connection_.probe_buffer.size() >= 8) {
        connection_.format = StreamFormat::TextDelimited;
        ESP_LOGI(kTag, "Detected text/csv stream");
    }
}

void DataProcessor::process_binary_buffer() {
    while (connection_.binary_buffer.size() >= 2) {
        if (connection_.binary_buffer[0] != kHeader0 || connection_.binary_buffer[1] != kHeader1) {
            static constexpr std::array<uint8_t, 2> kSyncWord = {kHeader0, kHeader1};
            auto sync_it = std::search(connection_.binary_buffer.begin() + 1,
                                       connection_.binary_buffer.end(),
                                       kSyncWord.begin(),
                                       kSyncWord.end());
            if (sync_it == connection_.binary_buffer.end()) {
                connection_.binary_buffer.clear();
                return;
            }
            connection_.binary_buffer.erase(connection_.binary_buffer.begin(), sync_it);
        }

        if (connection_.binary_buffer.size() < kBinaryFrameSize) {
            return;
        }

        process_binary_frame(connection_.binary_buffer.data());
        connection_.binary_buffer.erase(connection_.binary_buffer.begin(),
                                        connection_.binary_buffer.begin() + static_cast<std::ptrdiff_t>(kBinaryFrameSize));
    }
}

void DataProcessor::process_text_buffer() {
    while (true) {
        const size_t pos = connection_.text_buffer.find('\n');
        if (pos == std::string::npos) {
            return;
        }

        std::string line = connection_.text_buffer.substr(0, pos);
        connection_.text_buffer.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        process_text_line(line);
    }
}

void DataProcessor::process_binary_frame(const uint8_t *frame) {
    if (frame[0] != kHeader0 || frame[1] != kHeader1) {
        return;
    }
    if (calc_crc(frame, kBinaryFrameSize - 1) != frame[kBinaryFrameSize - 1]) {
        taskENTER_CRITICAL(&g_data_lock);
        ++crc_error_count_;
        taskEXIT_CRITICAL(&g_data_lock);
        ESP_LOGW(kTag, "Binary frame CRC mismatch");
        return;
    }

    const uint16_t seq = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    std::array<float, kChannels> values{};
    std::memcpy(values.data(), &frame[4], kChannels * sizeof(float));
    process_sample(values, true, seq);
}

void DataProcessor::process_text_line(const std::string &line) {
    std::array<float, kChannels> values{};
    if (parse_text_channels(line, values)) {
        process_sample(values, false, 0);
    } else {
        taskENTER_CRITICAL(&g_data_lock);
        ++text_parse_error_count_;
        taskEXIT_CRITICAL(&g_data_lock);
    }
}

bool DataProcessor::parse_text_channels(const std::string &line, std::array<float, kChannels> &values) const {
    const std::string trimmed = trim_copy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
        return false;
    }

    if (trimmed.find(',') != std::string::npos && split_csv_channels(trimmed, values)) {
        return true;
    }

    std::string normalized = trimmed;
    for (char &ch : normalized) {
        if (ch == ',' || ch == ';' || ch == '\t') {
            ch = ' ';
        }
    }

    std::vector<float> parsed;
    std::string token;
    for (size_t i = 0; i <= normalized.size(); ++i) {
        const char ch = (i < normalized.size()) ? normalized[i] : ' ';
        if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
            if (!token.empty()) {
                float value = 0.0f;
                if (parse_float_str(token, value)) {
                    parsed.push_back(value);
                }
                token.clear();
            }
        } else {
            token.push_back(ch);
        }
    }

    if (parsed.size() < kChannels) {
        return false;
    }

    for (int i = 0; i < kChannels; ++i) {
        values[i] = parsed[static_cast<size_t>(i)];
    }
    return true;
}

float DataProcessor::apply_filters(int channel, float sample) {
    if (!config_.filter.enabled) {
        return sample;
    }

    float out = sample;
    if (config_.filter.high_pass_enabled) {
        HighPassState &state = high_pass_[static_cast<size_t>(channel)];
        const float dt = 1.0f / static_cast<float>(config_.sample_rate_hz);
        const float rc = 1.0f / (2.0f * kPi * std::max(config_.filter.high_pass_cutoff_hz, 0.01f));
        const float alpha = rc / (rc + dt);
        out = alpha * (state.previous_output + out - state.previous_input);
        state.previous_input = sample;
        state.previous_output = out;
    }

    if (config_.filter.notch_enabled) {
        BiquadState &state = notch_[static_cast<size_t>(channel)];
        const float filtered = state.b0 * out + state.b1 * state.x1 + state.b2 * state.x2
                               - state.a1 * state.y1 - state.a2 * state.y2;
        state.x2 = state.x1;
        state.x1 = out;
        state.y2 = state.y1;
        state.y1 = filtered;
        out = filtered;
    }

    return out;
}

bool DataProcessor::should_run_detection() const {
    return samples_received_ >= kWindowSamples
           && (samples_received_ - last_detection_sample_) >= static_cast<uint32_t>(kDetectionStrideSamples);
}

int DataProcessor::selected_channel_count() const {
    return static_cast<int>(std::count(config_.channel_enabled.begin(), config_.channel_enabled.end(), true));
}

void DataProcessor::process_sample(const std::array<float, kChannels> &values, bool has_seq, uint16_t seq) {
    taskENTER_CRITICAL(&g_data_lock);
    const size_t ring_index = samples_received_ % kWindowSamples;
    for (int ch = 0; ch < kChannels; ++ch) {
        ring_[static_cast<size_t>(ch)][ring_index] = apply_filters(ch, values[static_cast<size_t>(ch)]);
    }

    ++samples_received_;
    last_rx_timestamp_ms_ = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    if (has_seq) {
        last_binary_seq_ = seq;
        has_binary_seq_ = true;
    }

    const bool run_detection = should_run_detection() && selected_channel_count() > 0;
    taskEXIT_CRITICAL(&g_data_lock);
    if (!run_detection) {
        return;
    }

    DetectionResult result = compute_cca();
    taskENTER_CRITICAL(&g_data_lock);
    last_detection_sample_ = samples_received_;
    if (!result.valid) {
        taskEXIT_CRITICAL(&g_data_lock);
        return;
    }
    ++detection_count_;
    last_detection_result_ = result;
    taskEXIT_CRITICAL(&g_data_lock);

    if (!spi_link_send_result(result)) {
        ESP_LOGW(kTag, "Failed to send SPI result packet");
    }
}

DetectionResult DataProcessor::compute_cca() {
    DetectionResult result = {};
    taskENTER_CRITICAL(&g_data_lock);
    result.sample_counter = samples_received_;
    result.last_binary_seq = has_binary_seq_ ? last_binary_seq_ : 0;
    const uint32_t local_samples_received = samples_received_;
    const int local_sample_rate = config_.sample_rate_hz;
    const std::array<uint16_t, 4> local_target_freq_hz = config_.target_freq_hz;
    const std::array<bool, kChannels> local_channel_enabled = config_.channel_enabled;
    taskEXIT_CRITICAL(&g_data_lock);
    result.timestamp_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

    const int selected = static_cast<int>(std::count(local_channel_enabled.begin(), local_channel_enabled.end(), true));
    const int ref_dims = kReferenceHarmonics * 2;
    const int samples = kWindowSamples;
    const float sample_rate = static_cast<float>(local_sample_rate);
    const uint32_t start_sample = local_samples_received - kWindowSamples;

    std::vector<float> x(static_cast<size_t>(selected * samples), 0.0f);
    std::vector<float> y(static_cast<size_t>(ref_dims * samples), 0.0f);

    int x_row = 0;
    for (int ch = 0; ch < kChannels; ++ch) {
        if (!local_channel_enabled[static_cast<size_t>(ch)]) {
            continue;
        }

        float mean = 0.0f;
        for (int s = 0; s < samples; ++s) {
            const size_t ring_index = (start_sample + static_cast<uint32_t>(s)) % kWindowSamples;
            mean += ring_[static_cast<size_t>(ch)][ring_index];
        }
        mean /= static_cast<float>(samples);

        for (int s = 0; s < samples; ++s) {
            const size_t ring_index = (start_sample + static_cast<uint32_t>(s)) % kWindowSamples;
            x[static_cast<size_t>(x_row * samples + s)] = ring_[static_cast<size_t>(ch)][ring_index] - mean;
        }
        ++x_row;
    }

    float best_corr = -1.0f;
    uint8_t best_index = 0xFF;

    for (size_t freq_idx = 0; freq_idx < config_.target_freq_hz.size(); ++freq_idx) {
        const float freq_hz = static_cast<float>(local_target_freq_hz[freq_idx]);

        for (int h = 0; h < kReferenceHarmonics; ++h) {
            const float harmonic = static_cast<float>(h + 1);
            float mean_sin = 0.0f;
            float mean_cos = 0.0f;
            for (int s = 0; s < samples; ++s) {
                const float t = static_cast<float>(s) / sample_rate;
                mean_sin += std::sin(2.0f * kPi * freq_hz * harmonic * t);
                mean_cos += std::cos(2.0f * kPi * freq_hz * harmonic * t);
            }
            mean_sin /= static_cast<float>(samples);
            mean_cos /= static_cast<float>(samples);

            for (int s = 0; s < samples; ++s) {
                const float t = static_cast<float>(s) / sample_rate;
                y[static_cast<size_t>((2 * h) * samples + s)] =
                    std::sin(2.0f * kPi * freq_hz * harmonic * t) - mean_sin;
                y[static_cast<size_t>((2 * h + 1) * samples + s)] =
                    std::cos(2.0f * kPi * freq_hz * harmonic * t) - mean_cos;
            }
        }

        std::vector<float> xt = transpose_matrix(x, selected, samples);
        std::vector<float> yt = transpose_matrix(y, ref_dims, samples);
        std::vector<float> sxx = multiply_matrix(x, xt, selected, samples, selected);
        std::vector<float> syy = multiply_matrix(y, yt, ref_dims, samples, ref_dims);
        std::vector<float> sxy = multiply_matrix(x, yt, selected, samples, ref_dims);
        std::vector<float> syx = transpose_matrix(sxy, selected, ref_dims);

        const float regularizer = 1e-3f;
        for (int i = 0; i < selected; ++i) {
            sxx[static_cast<size_t>(i * selected + i)] += regularizer;
        }
        for (int i = 0; i < ref_dims; ++i) {
            syy[static_cast<size_t>(i * ref_dims + i)] += regularizer;
        }

        std::vector<float> inv_sxx;
        std::vector<float> inv_syy;
        if (!invert_matrix(sxx, inv_sxx, selected) || !invert_matrix(syy, inv_syy, ref_dims)) {
            continue;
        }

        const std::vector<float> temp1 = multiply_matrix(inv_syy, syx, ref_dims, ref_dims, selected);
        const std::vector<float> temp2 = multiply_matrix(temp1, inv_sxx, ref_dims, selected, selected);
        const std::vector<float> cca_matrix = multiply_matrix(temp2, sxy, ref_dims, selected, ref_dims);

        const float eigen = dominant_eigenvalue_power_iteration(cca_matrix, ref_dims);
        const float corr = std::sqrt(std::max(eigen, 0.0f));
        result.correlations[freq_idx] = corr;

        if (corr > best_corr) {
            best_corr = corr;
            best_index = static_cast<uint8_t>(freq_idx);
        }
    }

    if (best_index != 0xFF && best_corr >= kDetectionThreshold) {
        result.valid = true;
        result.detected_index = best_index;
        result.detected_hz = local_target_freq_hz[best_index];
        result.confidence = best_corr;
        ESP_LOGI(kTag,
                 "CCA detected %u Hz idx=%u conf=%.3f [%.3f %.3f %.3f %.3f]",
                 result.detected_hz,
                 result.detected_index,
                 result.confidence,
                 result.correlations[0],
                 result.correlations[1],
                 result.correlations[2],
                 result.correlations[3]);
    }

    return result;
}

void DataProcessor::get_status(DataProcessStatus &out_status) const {
    out_status.tcp_listening = tcp_listening_;
    out_status.client_connected = client_connected_;
    out_status.stream_format = to_public_stream_format(connection_.format);
    std::snprintf(out_status.client_ip, sizeof(out_status.client_ip), "%s", client_ip_);
    out_status.client_port = client_port_;
    out_status.samples_received = samples_received_;
    out_status.last_rx_timestamp_ms = last_rx_timestamp_ms_;
    out_status.detection_count = detection_count_;
    out_status.crc_error_count = crc_error_count_;
    out_status.text_parse_error_count = text_parse_error_count_;
    out_status.has_binary_seq = has_binary_seq_;
    out_status.last_binary_seq = last_binary_seq_;
    out_status.last_detection = last_detection_result_;
}

void DataProcessor::get_config(DataProcessConfigSnapshot &out_config) const {
    out_config.channel_enabled = config_.channel_enabled;
    out_config.sample_rate_hz = config_.sample_rate_hz;
    out_config.target_freq_hz = config_.target_freq_hz;
    out_config.filter_enabled = config_.filter.enabled;
    out_config.high_pass_enabled = config_.filter.high_pass_enabled;
    out_config.high_pass_cutoff_hz = config_.filter.high_pass_cutoff_hz;
    out_config.notch_enabled = config_.filter.notch_enabled;
    out_config.notch_hz = config_.filter.notch_hz;
    out_config.notch_q = config_.filter.notch_q;
}

bool DataProcessor::set_sample_rate(int sample_rate_hz) {
    if (sample_rate_hz < 64 || sample_rate_hz > 4000) {
        return false;
    }
    config_.sample_rate_hz = sample_rate_hz;
    init_filter_states();
    return true;
}

bool DataProcessor::set_filter_enabled(bool enabled) {
    config_.filter.enabled = enabled;
    init_filter_states();
    return true;
}

bool DataProcessor::set_high_pass(bool enabled, float cutoff_hz) {
    if (cutoff_hz <= 0.0f || cutoff_hz >= (static_cast<float>(config_.sample_rate_hz) * 0.45f)) {
        return false;
    }
    config_.filter.high_pass_enabled = enabled;
    config_.filter.high_pass_cutoff_hz = cutoff_hz;
    init_filter_states();
    return true;
}

bool DataProcessor::set_notch(bool enabled, float notch_hz, float notch_q) {
    if (notch_hz <= 0.0f || notch_hz >= (static_cast<float>(config_.sample_rate_hz) * 0.45f) || notch_q <= 0.1f) {
        return false;
    }
    config_.filter.notch_enabled = enabled;
    config_.filter.notch_hz = notch_hz;
    config_.filter.notch_q = notch_q;
    init_filter_states();
    return true;
}

bool DataProcessor::set_channels_from_list(const char *channel_list) {
    if (channel_list == nullptr) {
        return false;
    }

    std::array<bool, kChannels> new_mask = {false, false, false, false, false, false, false, false};
    std::string text = trim_copy(channel_list);
    if (text == "all") {
        new_mask.fill(true);
    } else if (text == "none") {
        return false;
    } else {
        size_t start = 0;
        while (start <= text.size()) {
            const size_t comma = text.find(',', start);
            const size_t end = (comma == std::string::npos) ? text.size() : comma;
            const std::string token = trim_copy(text.substr(start, end - start));
            if (token.empty()) {
                return false;
            }

            char *parse_end = nullptr;
            const long channel = std::strtol(token.c_str(), &parse_end, 10);
            if (parse_end == token.c_str() || *parse_end != '\0' || channel < 1 || channel > kChannels) {
                return false;
            }
            new_mask[static_cast<size_t>(channel - 1)] = true;

            if (comma == std::string::npos) {
                break;
            }
            start = comma + 1;
        }
    }

    if (std::count(new_mask.begin(), new_mask.end(), true) == 0) {
        return false;
    }

    config_.channel_enabled = new_mask;
    return true;
}

void data_process_task(void *arg) {
    auto *processor = static_cast<DataProcessor *>(arg);
    processor->run();
}

DataProcessor g_processor;
TaskHandle_t g_task = nullptr;

}  // namespace

void data_process_start() {
    if (g_task != nullptr) {
        return;
    }

    xTaskCreate(data_process_task, "data_process_task", 12288, &g_processor, 5, &g_task);
}

bool data_process_get_status(DataProcessStatus *out_status) {
    if (out_status == nullptr) {
        return false;
    }
    out_status->task_started = (g_task != nullptr);
    taskENTER_CRITICAL(&g_data_lock);
    g_processor.get_status(*out_status);
    taskEXIT_CRITICAL(&g_data_lock);
    return true;
}

bool data_process_get_config(DataProcessConfigSnapshot *out_config) {
    if (out_config == nullptr) {
        return false;
    }
    taskENTER_CRITICAL(&g_data_lock);
    g_processor.get_config(*out_config);
    taskEXIT_CRITICAL(&g_data_lock);
    return true;
}

bool data_process_set_sample_rate(int sample_rate_hz) {
    taskENTER_CRITICAL(&g_data_lock);
    const bool ok = g_processor.set_sample_rate(sample_rate_hz);
    taskEXIT_CRITICAL(&g_data_lock);
    return ok;
}

bool data_process_set_filter_enabled(bool enabled) {
    taskENTER_CRITICAL(&g_data_lock);
    const bool ok = g_processor.set_filter_enabled(enabled);
    taskEXIT_CRITICAL(&g_data_lock);
    return ok;
}

bool data_process_set_high_pass(bool enabled, float cutoff_hz) {
    taskENTER_CRITICAL(&g_data_lock);
    const bool ok = g_processor.set_high_pass(enabled, cutoff_hz);
    taskEXIT_CRITICAL(&g_data_lock);
    return ok;
}

bool data_process_set_notch(bool enabled, float notch_hz, float notch_q) {
    taskENTER_CRITICAL(&g_data_lock);
    const bool ok = g_processor.set_notch(enabled, notch_hz, notch_q);
    taskEXIT_CRITICAL(&g_data_lock);
    return ok;
}

bool data_process_set_channels_from_list(const char *channel_list) {
    taskENTER_CRITICAL(&g_data_lock);
    const bool ok = g_processor.set_channels_from_list(channel_list);
    taskEXIT_CRITICAL(&g_data_lock);
    return ok;
}

void data_process_reset_runtime() {
    g_processor.external_reset_runtime();
}
