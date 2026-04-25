#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "linenoise/linenoise.h"
#include "lwip/inet.h"
#include "nvs_flash.h"

namespace {

constexpr int kMaxScanResults = 20;
constexpr int kConsoleHistoryLength = 16;
constexpr int kConsoleMaxLineLength = 128;
constexpr uart_port_t kConsoleUart = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);

static const char *TAG = "data_process";

enum ConsoleState {
    CONSOLE_STATE_COMMAND,
    CONSOLE_STATE_WAIT_WIFI_INDEX,
    CONSOLE_STATE_WAIT_WIFI_PASSWORD,
};

struct WifiConsoleContext {
    wifi_ap_record_t scan_results[kMaxScanResults];
    uint16_t scan_count = 0;
    int selected_index = -1;
    ConsoleState state = CONSOLE_STATE_COMMAND;
    bool wifi_started = false;
    bool has_scan_results = false;
    char selected_ssid[33] = {0};
};

static WifiConsoleContext g_ctx;
static EventGroupHandle_t g_wifi_event_group = nullptr;
static bool g_wifi_stack_ready = false;

constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
constexpr EventBits_t WIFI_GOT_IP_BIT = BIT1;
static char g_prompt[16] = "cmd> ";

static void init_console()
{
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

static const char *current_prompt()
{
    switch (g_ctx.state) {
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

static void print_prompt_hint()
{
    if (g_ctx.state == CONSOLE_STATE_WAIT_WIFI_PASSWORD) {
        printf("Enter password for \"%s\".\n", g_ctx.selected_ssid);
    }
    fflush(stdout);
}

static void trim_line(char *line)
{
    size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
}

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            g_ctx.wifi_started = true;
            break;
        case WIFI_EVENT_STA_CONNECTED:
            xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(g_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
            break;
        default:
            break;
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        printf("Wi-Fi got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(g_wifi_event_group, WIFI_GOT_IP_BIT);
    }
}

static bool ensure_wifi_stack_ready()
{
    if (g_wifi_stack_ready) {
        return true;
    }

    esp_err_t err = ESP_OK;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }

    if (g_wifi_event_group == nullptr) {
        g_wifi_event_group = xEventGroupCreate();
        if (g_wifi_event_group == nullptr) {
            ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
            return false;
        }
    }

    if (!g_wifi_stack_ready) {
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        if (sta_netif == nullptr) {
            ESP_LOGE(TAG, "Failed to create STA netif");
            return false;
        }

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
            return false;
        }

        esp_event_handler_instance_t wifi_instance = nullptr;
        esp_event_handler_instance_t ip_instance = nullptr;

        err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, &wifi_instance);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register WIFI_EVENT handler failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &ip_instance);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register IP_EVENT handler failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
            return false;
        }

        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
            return false;
        }

        g_wifi_stack_ready = true;
        ESP_LOGI(TAG, "Wi-Fi stack initialized through ESP32-C5 hosted link");
    }

    return true;
}

static void print_help()
{
    printf("\nAvailable commands:\n");
    printf("  help      - show this help\n");
    printf("  wifiinit  - initialize hosted Wi-Fi stack\n");
    printf("  scan      - scan Wi-Fi and print the list\n");
    printf("  status    - print current Wi-Fi status\n");
    printf("  disconnect- disconnect current Wi-Fi\n");
    printf("  reboot    - restart the chip\n");
}

static void print_status()
{
    if (g_wifi_event_group == nullptr) {
        printf("\nWi-Fi event group not initialized yet.\n");
        printf("Run 'wifiinit' first.\n");
        return;
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_ap_record_t ap_info = {};
    esp_err_t ap_ret = esp_wifi_sta_get_ap_info(&ap_info);

    ESP_ERROR_CHECK(esp_wifi_get_mode(&mode));

    printf("\nWi-Fi started: %s\n", g_ctx.wifi_started ? "yes" : "no");
    printf("Mode: %d\n", static_cast<int>(mode));
    printf("Connected: %s\n", (xEventGroupGetBits(g_wifi_event_group) & WIFI_CONNECTED_BIT) ? "yes" : "no");
    printf("Got IP: %s\n", (xEventGroupGetBits(g_wifi_event_group) & WIFI_GOT_IP_BIT) ? "yes" : "no");

    if (ap_ret == ESP_OK) {
        printf("Current SSID: %s\n", reinterpret_cast<const char *>(ap_info.ssid));
        printf("RSSI: %d dBm\n", ap_info.rssi);
        printf("Channel: %d\n", ap_info.primary);
    }
}

static void print_scan_results()
{
    printf("\nFound %u AP(s):\n", g_ctx.scan_count);
    for (uint16_t i = 0; i < g_ctx.scan_count; ++i) {
        const wifi_ap_record_t &ap = g_ctx.scan_results[i];
        printf("  [%u] SSID: %s | RSSI: %d | CH: %u | auth: %d\n",
               i,
               reinterpret_cast<const char *>(ap.ssid),
               ap.rssi,
               ap.primary,
               static_cast<int>(ap.authmode));
    }
}

static void start_scan()
{
    if (!ensure_wifi_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link and try again.\n");
        return;
    }

    uint16_t ap_count = kMaxScanResults;
    wifi_scan_config_t scan_cfg = {};

    ESP_LOGI(TAG, "Starting Wi-Fi scan");
    xEventGroupClearBits(g_wifi_event_group, WIFI_GOT_IP_BIT);

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, g_ctx.scan_results));

    g_ctx.scan_count = ap_count;
    g_ctx.has_scan_results = (ap_count > 0);

    print_scan_results();

    if (g_ctx.has_scan_results) {
        g_ctx.state = CONSOLE_STATE_WAIT_WIFI_INDEX;
        printf("\nEnter the Wi-Fi index you want to connect.\n");
    } else {
        printf("\nNo AP found.\n");
    }
}

static void connect_selected_ap(const char *password)
{
    if (!ensure_wifi_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link and try again.\n");
        return;
    }

    if (g_ctx.selected_index < 0 || g_ctx.selected_index >= g_ctx.scan_count) {
        printf("No Wi-Fi selected.\n");
        return;
    }

    wifi_config_t wifi_config = {};
    const wifi_ap_record_t &ap = g_ctx.scan_results[g_ctx.selected_index];

    std::memcpy(wifi_config.sta.ssid, ap.ssid, sizeof(ap.ssid));
    std::snprintf(reinterpret_cast<char *>(wifi_config.sta.password),
                  sizeof(wifi_config.sta.password),
                  "%s",
                  password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    xEventGroupClearBits(g_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);

    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGE(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        return;
    }

    printf("Connecting to \"%s\"...\n", g_ctx.selected_ssid);
}

static void handle_command(char *line)
{
    if (std::strcmp(line, "help") == 0) {
        print_help();
    } else if (std::strcmp(line, "wifiinit") == 0) {
        if (ensure_wifi_stack_ready()) {
            printf("Wi-Fi stack initialized.\n");
        } else {
            printf("Wi-Fi init failed. Check C5 firmware/SDIO link.\n");
        }
    } else if (std::strcmp(line, "scan") == 0) {
        start_scan();
    } else if (std::strcmp(line, "status") == 0) {
        print_status();
    } else if (std::strcmp(line, "disconnect") == 0) {
        if (!ensure_wifi_stack_ready()) {
            printf("Wi-Fi init failed. Check C5 firmware/SDIO link.\n");
            return;
        }

        esp_err_t err = esp_wifi_disconnect();
        if (err == ESP_OK || err == ESP_ERR_WIFI_NOT_CONNECT) {
            printf("Disconnected.\n");
        } else {
            ESP_LOGE(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(err));
        }
    } else if (std::strcmp(line, "reboot") == 0) {
        printf("Rebooting...\n");
        fflush(stdout);
        esp_restart();
    } else if (line[0] != '\0') {
        printf("Unknown command: %s\n", line);
        print_help();
    }
}

static void handle_wifi_index(char *line)
{
    char *end = nullptr;
    long index = std::strtol(line, &end, 10);

    if (end == line || *end != '\0' || index < 0 || index >= g_ctx.scan_count) {
        printf("Invalid index. Enter a number from 0 to %u.\n", g_ctx.scan_count - 1);
        return;
    }

    g_ctx.selected_index = static_cast<int>(index);
    std::snprintf(
        g_ctx.selected_ssid,
        sizeof(g_ctx.selected_ssid),
        "%s",
        reinterpret_cast<const char *>(g_ctx.scan_results[g_ctx.selected_index].ssid));

    g_ctx.state = CONSOLE_STATE_WAIT_WIFI_PASSWORD;
    printf("Selected SSID: %s\n", g_ctx.selected_ssid);
}

static void handle_wifi_password(char *line)
{
    connect_selected_ap(line);
    g_ctx.state = CONSOLE_STATE_COMMAND;
}

static void console_task(void *arg)
{
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
            g_ctx.state = CONSOLE_STATE_COMMAND;
            printf("Canceled.\n");
            linenoiseFree(line);
            continue;
        }

        switch (g_ctx.state) {
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
            g_ctx.state = CONSOLE_STATE_COMMAND;
            break;
        }

        if (line[0] != '\0') {
            linenoiseHistoryAdd(line);
        }
        linenoiseFree(line);
    }
}

}  // namespace

extern "C" void app_main()
{
    init_nvs();
    init_console();

    ESP_LOGI(TAG, "Pure ESP32-P4 data-processing framework is running");
    ESP_LOGI(TAG, "Use the serial console to initialize Wi-Fi, scan, and connect");

    xTaskCreate(console_task, "console_task", 8192, nullptr, 5, nullptr);

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
