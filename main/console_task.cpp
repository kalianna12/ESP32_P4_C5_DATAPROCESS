#include "console_task.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linenoise/linenoise.h"

#include "dataprocess.h"
#include "spi_link.h"
#include "wifi.h"

namespace {

constexpr int kConsoleHistoryLength = 16;
constexpr int kConsoleMaxLineLength = 128;
constexpr uart_port_t kConsoleUart = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);

enum ConsoleState {
    CONSOLE_STATE_COMMAND,
    CONSOLE_STATE_WAIT_WIFI_INDEX,
    CONSOLE_STATE_WAIT_WIFI_PASSWORD,
};

struct ConsoleContext {
    ConsoleState state = CONSOLE_STATE_COMMAND;
};

static ConsoleContext g_console_ctx;
static char g_prompt[16] = "cmd> ";

static void init_console() {
    fflush(stdout);
    fsync(fileno(stdout));

    uart_vfs_dev_port_set_rx_line_endings(kConsoleUart, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(kConsoleUart, ESP_LINE_ENDINGS_CRLF);

    uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
#if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
#elif SOC_UART_SUPPORT_XTAL_CLK
        .source_clk = UART_SCLK_XTAL,
#endif
        .flags = {},
    };

    ESP_ERROR_CHECK(uart_driver_install(kConsoleUart, 256, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kConsoleUart, &uart_config));
    uart_vfs_dev_use_driver(kConsoleUart);

    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
    fcntl(fileno(stdout), F_SETFL, 0);

    linenoiseSetMultiLine(1);
    linenoiseSetMaxLineLen(kConsoleMaxLineLength);
    linenoiseSetDumbMode(linenoiseProbe() != 0);
    linenoiseAllowEmpty(false);
    linenoiseHistorySetMaxLen(kConsoleHistoryLength);
}

static const char *current_prompt() {
    switch (g_console_ctx.state) {
    case CONSOLE_STATE_COMMAND:
        std::snprintf(g_prompt, sizeof(g_prompt), "cmd> ");
        break;
    case CONSOLE_STATE_WAIT_WIFI_INDEX:
        std::snprintf(g_prompt, sizeof(g_prompt), "wifi#> ");
        break;
    case CONSOLE_STATE_WAIT_WIFI_PASSWORD:
        std::snprintf(g_prompt, sizeof(g_prompt), "pass> ");
        break;
    default:
        std::snprintf(g_prompt, sizeof(g_prompt), "cmd> ");
        break;
    }
    return g_prompt;
}

static void print_prompt_hint() {
    if (g_console_ctx.state == CONSOLE_STATE_WAIT_WIFI_PASSWORD) {
        printf("Enter password for \"%s\".\n", wifi_get_selected_ssid());
    }
    fflush(stdout);
}

static void trim_line(char *line) {
    size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
}

static void print_help() {
    printf("\nAvailable commands:\n");
    // 使用 %-32s 让左边的命令固定占用 32 个字符宽度，不足补空格
    printf("  %-32s - %s\n", "help", "show this help");
    printf("  %-32s - %s\n", "wifiinit", "initialize hosted Wi-Fi stack");
    printf("  %-32s - %s\n", "scan", "scan Wi-Fi and print the list");
    printf("  %-32s - %s\n", "status", "print current Wi-Fi status");
    printf("  %-32s - %s\n", "disconnect", "disconnect current Wi-Fi");
    printf("  %-32s - %s\n", "stackinfo", "show remaining stack space of console task");
    printf("  %-32s - %s\n", "autoconnect", "auto-scan and connect to strongest saved Wi-Fi");
    printf("  %-32s - %s\n", "dpstatus", "show TCP/CCA runtime status");
    printf("  %-32s - %s\n", "dpcfg", "show current data-processing config");
    printf("  %-32s - %s\n", "dpreset", "clear current runtime counters/window");
    printf("  %-32s - %s\n", "dpsamplerate <hz>", "update CCA sample rate");
    printf("  %-32s - %s\n", "dpchannels <all|1,2,3,4>", "enable the selected channels");
    printf("  %-32s - %s\n", "dpfilter <on|off>", "enable or disable all software filtering");
    printf("  %-32s - %s\n", "dphp <on|off> [cutoff_hz]", "control the high-pass stage");
    printf("  %-32s - %s\n", "dpnotch <on|off> [freq_hz] [q]", "control the notch stage");
    printf("  %-32s - %s\n", "8 | 10 | 12 | 14", "send a SPI test result to the display P4");
    printf("  %-32s - %s\n", "reboot", "restart the chip");
}

static void print_stack_info() {
    UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(nullptr);
    printf("\nConsole task remaining stack: %u bytes\n", remaining_stack * sizeof(StackType_t));
    printf("(Stack high water mark: %u words)\n", remaining_stack);
}

static bool parse_on_off(const char *value, bool *enabled) {
    if (value == nullptr || enabled == nullptr) {
        return false;
    }
    if (std::strcmp(value, "on") == 0) {
        *enabled = true;
        return true;
    }
    if (std::strcmp(value, "off") == 0) {
        *enabled = false;
        return true;
    }
    return false;
}

static const char *stream_format_name(DataStreamFormat format) {
    switch (format) {
    case DataStreamFormat::BinaryAds1299:
        return "binary_ads1299";
    case DataStreamFormat::TextDelimited:
        return "text_csv";
    case DataStreamFormat::Unknown:
    default:
        return "unknown";
    }
}

static void print_data_process_status() {
    DataProcessStatus status = {};
    if (!data_process_get_status(&status)) {
        printf("Failed to read data-process status.\n");
        return;
    }

    printf("\n%-25s : %s\n", "Data process task started", status.task_started ? "yes" : "no");
    printf("%-25s : %s\n", "TCP listening", status.tcp_listening ? "yes" : "no");
    printf("%-25s : %s\n", "Client connected", status.client_connected ? "yes" : "no");
    if (status.client_connected || status.client_ip[0] != '\0') {
        printf("%-25s : %s:%u\n", "Client endpoint", status.client_ip[0] != '\0' ? status.client_ip : "-", status.client_port);
    }
    printf("%-25s : %s\n", "Detected stream format", stream_format_name(status.stream_format));
    printf("%-25s : %lu\n", "Samples received", static_cast<unsigned long>(status.samples_received));
    printf("%-25s : %lu ms\n", "Last RX timestamp", static_cast<unsigned long>(status.last_rx_timestamp_ms));
    printf("%-25s : %lu\n", "Detections sent", static_cast<unsigned long>(status.detection_count));
    printf("%-25s : %lu\n", "CRC errors", static_cast<unsigned long>(status.crc_error_count));
    printf("%-25s : %lu\n", "Text parse errors", static_cast<unsigned long>(status.text_parse_error_count));
    
    printf("%-25s : ", "Last binary seq");
    if (status.has_binary_seq) {
        printf("%u\n", status.last_binary_seq);
    } else {
        printf("n/a\n");
    }

    if (status.last_detection.valid) {
        printf("%-25s : %u Hz idx=%u conf=%.3f @ sample %lu\n", "Last detection",
               status.last_detection.detected_hz,
               status.last_detection.detected_index,
               status.last_detection.confidence,
               static_cast<unsigned long>(status.last_detection.sample_counter));
        printf("%-25s : [%.3f %.3f %.3f %.3f]\n", "Correlations",
               status.last_detection.correlations[0],
               status.last_detection.correlations[1],
               status.last_detection.correlations[2],
               status.last_detection.correlations[3]);
    } else {
        printf("%-25s : none yet\n", "Last detection");
    }
}

static void print_data_process_config() {
    DataProcessConfigSnapshot config = {};
    if (!data_process_get_config(&config)) {
        printf("Failed to read data-process config.\n");
        return;
    }

    printf("\n%-22s : %d Hz\n", "Sample rate", config.sample_rate_hz);
    printf("%-22s : %u, %u, %u, %u Hz\n", "Target freqs",
           config.target_freq_hz[0], config.target_freq_hz[1],
           config.target_freq_hz[2], config.target_freq_hz[3]);
    
    printf("%-22s :", "Channels enabled");
    for (size_t i = 0; i < config.channel_enabled.size(); ++i) {
        if (config.channel_enabled[i]) {
            printf(" %u", static_cast<unsigned>(i + 1));
        }
    }
    printf("\n");
    
    printf("%-22s : %s\n", "Filter enabled", config.filter_enabled ? "on" : "off");
    printf("%-22s : %s (cutoff %.2f Hz)\n", "High-pass", config.high_pass_enabled ? "on" : "off", config.high_pass_cutoff_hz);
    printf("%-22s : %s (freq %.2f Hz, Q %.2f)\n", "Notch", config.notch_enabled ? "on" : "off", config.notch_hz, config.notch_q);
}

static void handle_data_process_command(char *line) {
    if (std::strcmp(line, "dpstatus") == 0) {
        print_data_process_status();
        return;
    }
    if (std::strcmp(line, "dpcfg") == 0) {
        print_data_process_config();
        return;
    }
    if (std::strcmp(line, "dpreset") == 0) {
        data_process_reset_runtime();
        printf("Data-process runtime state cleared.\n");
        return;
    }
    if (std::strncmp(line, "dpsamplerate ", 13) == 0) {
        char *end = nullptr;
        const long sample_rate = std::strtol(line + 13, &end, 10);
        if (end == line + 13 || *end != '\0' || !data_process_set_sample_rate(static_cast<int>(sample_rate))) {
            printf("Usage: dpsamplerate <64..4000>\n");
            return;
        }
        printf("Sample rate updated to %ld Hz.\n", sample_rate);
        return;
    }
    if (std::strncmp(line, "dpchannels ", 11) == 0) {
        if (!data_process_set_channels_from_list(line + 11)) {
            printf("Usage: dpchannels <all|1,2,3,4>\n");
            return;
        }
        printf("Channel selection updated.\n");
        return;
    }
    if (std::strncmp(line, "dpfilter ", 9) == 0) {
        bool enabled = false;
        if (!parse_on_off(line + 9, &enabled) || !data_process_set_filter_enabled(enabled)) {
            printf("Usage: dpfilter <on|off>\n");
            return;
        }
        printf("Software filter %s.\n", enabled ? "enabled" : "disabled");
        return;
    }
    if (std::strncmp(line, "dphp ", 5) == 0) {
        char mode[8] = {0};
        float cutoff_hz = 1.0f;
        const int parsed = std::sscanf(line + 5, "%7s %f", mode, &cutoff_hz);
        bool enabled = false;
        if (parsed < 1 || !parse_on_off(mode, &enabled)) {
            printf("Usage: dphp <on|off> [cutoff_hz]\n");
            return;
        }
        if (!data_process_set_high_pass(enabled, cutoff_hz)) {
            printf("High-pass update failed. Check cutoff against current sample rate.\n");
            return;
        }
        printf("High-pass %s at %.2f Hz.\n", enabled ? "enabled" : "disabled", cutoff_hz);
        return;
    }
    if (std::strncmp(line, "dpnotch ", 8) == 0) {
        char mode[8] = {0};
        float notch_hz = 50.0f;
        float q = 30.0f;
        const int parsed = std::sscanf(line + 8, "%7s %f %f", mode, &notch_hz, &q);
        bool enabled = false;
        if (parsed < 1 || !parse_on_off(mode, &enabled)) {
            printf("Usage: dpnotch <on|off> [freq_hz] [q]\n");
            return;
        }
        if (!data_process_set_notch(enabled, notch_hz, q)) {
            printf("Notch update failed. Check freq/Q against current sample rate.\n");
            return;
        }
        printf("Notch %s at %.2f Hz, Q=%.2f.\n", enabled ? "enabled" : "disabled", notch_hz, q);
        return;
    }

    printf("Unknown data-process command: %s\n", line);
    printf("Run 'help' to see the available dp* commands.\n");
}

static void handle_command(char *line) {
    if (std::strcmp(line, "help") == 0) {
        print_help();
    } else if (std::strcmp(line, "8") == 0 ||
               std::strcmp(line, "10") == 0 ||
               std::strcmp(line, "12") == 0 ||
               std::strcmp(line, "14") == 0) {
        const uint16_t detected_hz = static_cast<uint16_t>(std::strtoul(line, nullptr, 10));
        if (spi_link_send_test_frequency(detected_hz)) {
            printf("SPI test packet sent: %u Hz\n", detected_hz);
        } else {
            printf("Failed to send SPI test packet: %u Hz\n", detected_hz);
        }
    } else if (std::strcmp(line, "wifiinit") == 0) {
        if (wifi_ensure_stack_ready()) {
            printf("Wi-Fi stack initialized.\n");
        } else {
            printf("Wi-Fi init failed. Check C5 firmware/SDIO link.\n");
        }
    } else if (std::strcmp(line, "scan") == 0) {
        wifi_start_scan();
        if (wifi_has_scan_results()) {
            g_console_ctx.state = CONSOLE_STATE_WAIT_WIFI_INDEX;
        }
    } else if (std::strcmp(line, "status") == 0) {
        wifi_print_status();
    } else if (std::strcmp(line, "stackinfo") == 0) {
        print_stack_info();
    } else if (std::strcmp(line, "autoconnect") == 0) {
        wifi_autoconnect();
    } else if (std::strcmp(line, "disconnect") == 0) {
        wifi_disconnect_current();
    } else if (std::strcmp(line, "reboot") == 0) {
        printf("Rebooting...\n");
        fflush(stdout);
        esp_restart();
    } else if (std::strncmp(line, "dp", 2) == 0) {
        handle_data_process_command(line);
    } else if (line[0] != '\0') {
        printf("Unknown command: %s\n", line);
        print_help();
    }
}

static void handle_wifi_index(char *line) {
    if (std::strcmp(line, "scan") == 0 || std::strcmp(line, "rescan") == 0) {
        wifi_start_scan();
        if (!wifi_has_scan_results()) {
            g_console_ctx.state = CONSOLE_STATE_COMMAND;
        }
        return;
    }

    char *end = nullptr;
    long index = std::strtol(line, &end, 10);

    if (end == line || *end != '\0' || index < 0 || index >= wifi_get_scan_count()) {
        const uint16_t max_index = wifi_get_scan_count() > 0 ? wifi_get_scan_count() - 1 : 0;
        printf("Invalid index. Enter a number from 0 to %u, or type 'scan' to scan again.\n", max_index);
        return;
    }

    if (!wifi_select_ap_by_index(static_cast<int>(index))) {
        printf("Failed to select Wi-Fi index.\n");
        return;
    }

    g_console_ctx.state = CONSOLE_STATE_WAIT_WIFI_PASSWORD;
    printf("Selected SSID: %s\n", wifi_get_selected_ssid());
}

static void handle_wifi_password(char *line) {
    wifi_connect_selected_ap(line);
    g_console_ctx.state = CONSOLE_STATE_COMMAND;
}

static void console_task_entry(void *arg) {
    (void)arg;

    printf("\nESP32-P4 data-processing console ready.\n");
    print_help();

    while (true) {
        print_prompt_hint();
        char *line = linenoise(current_prompt());
        if (line == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        trim_line(line);

        if (std::strcmp(line, "cancel") == 0) {
            g_console_ctx.state = CONSOLE_STATE_COMMAND;
            printf("Canceled.\n");
            linenoiseFree(line);
            continue;
        }

        switch (g_console_ctx.state) {
        case CONSOLE_STATE_COMMAND:
            handle_command(line);
            break;
        case CONSOLE_STATE_WAIT_WIFI_INDEX:
            handle_wifi_index(line);
            break;
        case CONSOLE_STATE_WAIT_WIFI_PASSWORD:
            handle_wifi_password(line);
            break;
        default:
            g_console_ctx.state = CONSOLE_STATE_COMMAND;
            break;
        }

        if (line[0] != '\0') {
            linenoiseHistoryAdd(line);
        }
        linenoiseFree(line);
    }
}

}  // namespace

void console_task_start() {
    init_console();
    xTaskCreate(console_task_entry, "console_task", 8192, nullptr, 5, nullptr);
}
