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
    printf("  help       - show this help\n");
    printf("  wifiinit   - initialize hosted Wi-Fi stack\n");
    printf("  scan       - scan Wi-Fi and print the list\n");
    printf("  status     - print current Wi-Fi status\n");
    printf("  disconnect - disconnect current Wi-Fi\n");
    printf("  stackinfo  - show remaining stack space of console task\n");
    printf("  autoconnect- auto-scan and connect to strongest saved Wi-Fi\n");
    printf("  reboot     - restart the chip\n");
}

static void print_stack_info() {
    UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(nullptr);
    printf("\nConsole task remaining stack: %u bytes\n", remaining_stack * sizeof(StackType_t));
    printf("(Stack high water mark: %u words)\n", remaining_stack);
}

static void handle_command(char *line) {
    if (std::strcmp(line, "help") == 0) {
        print_help();
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
