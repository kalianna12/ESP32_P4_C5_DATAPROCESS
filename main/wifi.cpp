#include "wifi.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#ifndef CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM
#define CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM 10
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM
#define CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM 32
#endif
#ifndef CONFIG_WIFI_RMT_TX_BUFFER_TYPE
#define CONFIG_WIFI_RMT_TX_BUFFER_TYPE 1
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_RX_MGMT_BUF
#define CONFIG_WIFI_RMT_DYNAMIC_RX_MGMT_BUF 0
#endif
#ifndef CONFIG_WIFI_RMT_ESPNOW_MAX_ENCRYPT_NUM
#define CONFIG_WIFI_RMT_ESPNOW_MAX_ENCRYPT_NUM 7
#endif

#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {

constexpr int kMaxScanResults = 32;
constexpr int kMaxSavedWifi = 10;
constexpr const char *kNvsNamespace = "wifi_config";
constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
constexpr EventBits_t WIFI_GOT_IP_BIT = BIT1;

struct SavedWifiConfig {
    char ssid[33];
    char password[65];
};

struct WifiContext {
    wifi_ap_record_t scan_results[kMaxScanResults];
    uint16_t scan_count = 0;
    int selected_index = -1;
    bool wifi_started = false;
    bool has_scan_results = false;
    char selected_ssid[33] = {0};
    SavedWifiConfig saved_wifi[kMaxSavedWifi];
    uint16_t saved_wifi_count = 0;
};

static const char *TAG = "wifi";
static WifiContext g_ctx;
static EventGroupHandle_t g_wifi_event_group = nullptr;
static bool g_wifi_stack_ready = false;
static char g_connecting_ssid[33] = {0};
static char g_connecting_password[65] = {0};
static bool g_should_save_wifi = false;

static esp_err_t save_wifi_to_nvs(const char *ssid, const char *password);
static esp_err_t load_saved_wifi();
static void connect_to_ap(const char *ssid, const char *password, bool save_to_nvs);

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
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

        if (g_should_save_wifi && g_connecting_ssid[0] != '\0') {
            esp_err_t err = save_wifi_to_nvs(g_connecting_ssid, g_connecting_password);
            if (err == ESP_OK) {
                printf("Wi-Fi credentials saved to flash.\n");
                load_saved_wifi();
            }
            g_should_save_wifi = false;
            g_connecting_ssid[0] = '\0';
            g_connecting_password[0] = '\0';
        }
    }
}

static void print_scan_results() {
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

static esp_err_t save_wifi_to_nvs(const char *ssid, const char *password) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t count = 0;
    nvs_get_u32(handle, "count", &count);

    bool found = false;
    for (uint32_t i = 0; i < count && i < kMaxSavedWifi; i++) {
        char key[32];
        SavedWifiConfig saved = {};
        std::snprintf(key, sizeof(key), "ssid_%lu", static_cast<unsigned long>(i));
        if (nvs_get_str(handle, key, nullptr, nullptr) == ESP_OK) {
            size_t len = sizeof(saved.ssid);
            nvs_get_str(handle, key, saved.ssid, &len);
            if (std::strcmp(saved.ssid, ssid) == 0) {
                std::snprintf(key, sizeof(key), "pass_%lu", static_cast<unsigned long>(i));
                nvs_set_str(handle, key, password);
                found = true;
                break;
            }
        }
    }

    if (!found && count < kMaxSavedWifi) {
        char key[32];
        std::snprintf(key, sizeof(key), "ssid_%lu", static_cast<unsigned long>(count));
        nvs_set_str(handle, key, ssid);
        std::snprintf(key, sizeof(key), "pass_%lu", static_cast<unsigned long>(count));
        nvs_set_str(handle, key, password);
        count++;
        nvs_set_u32(handle, "count", count);
    }

    nvs_commit(handle);
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t load_saved_wifi() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            g_ctx.saved_wifi_count = 0;
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t count = 0;
    nvs_get_u32(handle, "count", &count);
    g_ctx.saved_wifi_count = std::min(static_cast<uint32_t>(kMaxSavedWifi), count);

    for (uint16_t i = 0; i < g_ctx.saved_wifi_count; i++) {
        char key[32];
        std::snprintf(key, sizeof(key), "ssid_%u", i);
        size_t len = sizeof(g_ctx.saved_wifi[i].ssid);
        nvs_get_str(handle, key, g_ctx.saved_wifi[i].ssid, &len);

        std::snprintf(key, sizeof(key), "pass_%u", i);
        len = sizeof(g_ctx.saved_wifi[i].password);
        nvs_get_str(handle, key, g_ctx.saved_wifi[i].password, &len);
    }

    nvs_close(handle);
    return ESP_OK;
}

static void connect_to_ap(const char *ssid, const char *password, bool save_to_nvs) {
    if (!wifi_ensure_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link and try again.\n");
        return;
    }

    wifi_config_t wifi_config = {};
    std::memcpy(wifi_config.sta.ssid, reinterpret_cast<const uint8_t *>(ssid), std::strlen(ssid));
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

    if (save_to_nvs) {
        std::strncpy(g_connecting_ssid, ssid, sizeof(g_connecting_ssid) - 1);
        g_connecting_ssid[sizeof(g_connecting_ssid) - 1] = '\0';
        std::strncpy(g_connecting_password, password, sizeof(g_connecting_password) - 1);
        g_connecting_password[sizeof(g_connecting_password) - 1] = '\0';
        g_should_save_wifi = true;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        g_should_save_wifi = false;
        return;
    }

    printf("Connecting to \"%s\"...\n", ssid);
}

}  // namespace

void wifi_init_storage() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

bool wifi_ensure_stack_ready() {
    if (g_wifi_stack_ready) {
        return true;
    }

    esp_err_t err = esp_netif_init();
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
    // ==============================================================
    // [修复开始] 等待 C5 协处理器底层的 Wi-Fi 状态机真正就绪
    // ==============================================================
    ESP_LOGI(TAG, "Waiting for Coprocessor (C5) Wi-Fi station to start...");
    
    int wait_ms = 0;
    // 循环等待，直到 event_handler 将 wifi_started 设为 true，最多等 3 秒
    while (!g_ctx.wifi_started && wait_ms < 3000) {
        vTaskDelay(pdMS_TO_TICKS(50));
        wait_ms += 50;
    }

    if (!g_ctx.wifi_started) {
        ESP_LOGE(TAG, "C5 Wi-Fi failed to start in time. Please check SDIO link or reboot.");
        return false;
    }

    // 【极其关键的延时】
    // 即便收到了 start 事件，C5 的底层网络队列仍需时间初始化完毕。
    // 多给 200ms 的喘息时间，防止紧接着的 Scan 或 Connect 指令导致 C5 堵死丢包
    vTaskDelay(pdMS_TO_TICKS(200));
    // ==============================================================
    // [修复结束]
    // ==============================================================

    g_wifi_stack_ready = true;
    ESP_LOGI(TAG, "Wi-Fi stack initialized through ESP32-C5 hosted link");
    load_saved_wifi();
    return true;
}

bool wifi_is_connected_with_ip() {
    if (g_wifi_event_group == nullptr) {
        return false;
    }

    const EventBits_t bits = xEventGroupGetBits(g_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) && (bits & WIFI_GOT_IP_BIT);
}

void wifi_start_scan() {
    if (!wifi_ensure_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link and try again.\n");
        return;
    }

    g_ctx.selected_index = -1;
    g_ctx.selected_ssid[0] = '\0';
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
        printf("\nEnter the Wi-Fi index you want to connect.\n");
    } else {
        printf("\nNo AP found. Run 'scan' again when the network is available.\n");
    }
}

void wifi_print_status() {
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

void wifi_autoconnect() {
    if (!wifi_ensure_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link and try again.\n");
        return;
    }

    if (g_ctx.saved_wifi_count == 0) {
        printf("No saved Wi-Fi networks found.\n");
        return;
    }

    printf("Scanning for saved Wi-Fi networks...\n");
    uint16_t ap_count = kMaxScanResults;
    wifi_scan_config_t scan_cfg = {};

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, g_ctx.scan_results));

    int best_index = -1;
    int8_t best_rssi = -127;

    for (uint16_t i = 0; i < ap_count; i++) {
        const char *scan_ssid = reinterpret_cast<const char *>(g_ctx.scan_results[i].ssid);
        for (uint16_t j = 0; j < g_ctx.saved_wifi_count; j++) {
            if (std::strcmp(scan_ssid, g_ctx.saved_wifi[j].ssid) == 0) {
                if (g_ctx.scan_results[i].rssi > best_rssi) {
                    best_rssi = g_ctx.scan_results[i].rssi;
                    best_index = i;
                }
                break;
            }
        }
    }

    if (best_index >= 0) {
        const char *best_ssid = reinterpret_cast<const char *>(g_ctx.scan_results[best_index].ssid);
        for (uint16_t i = 0; i < g_ctx.saved_wifi_count; i++) {
            if (std::strcmp(best_ssid, g_ctx.saved_wifi[i].ssid) == 0) {
                printf("Found best saved network: %s (RSSI: %d dBm)\n", best_ssid, best_rssi);
                connect_to_ap(best_ssid, g_ctx.saved_wifi[i].password, false);
                return;
            }
        }
    }

    printf("No saved Wi-Fi networks detected in scan results.\n");
}

void wifi_disconnect_current() {
    if (!wifi_ensure_stack_ready()) {
        printf("Wi-Fi init failed. Check C5 firmware/SDIO link.\n");
        return;
    }

    esp_err_t err = esp_wifi_disconnect();
    if (err == ESP_OK || err == ESP_ERR_WIFI_NOT_CONNECT) {
        printf("Disconnected.\n");
    } else {
        ESP_LOGE(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(err));
    }
}

bool wifi_select_ap_by_index(int index) {
    if (index < 0 || index >= g_ctx.scan_count) {
        return false;
    }

    g_ctx.selected_index = index;
    std::snprintf(g_ctx.selected_ssid,
                  sizeof(g_ctx.selected_ssid),
                  "%s",
                  reinterpret_cast<const char *>(g_ctx.scan_results[g_ctx.selected_index].ssid));
    return true;
}

void wifi_connect_selected_ap(const char *password) {
    if (g_ctx.selected_index < 0 || g_ctx.selected_index >= g_ctx.scan_count) {
        printf("No Wi-Fi selected.\n");
        return;
    }

    const wifi_ap_record_t &ap = g_ctx.scan_results[g_ctx.selected_index];
    connect_to_ap(reinterpret_cast<const char *>(ap.ssid), password, true);
}

const char *wifi_get_selected_ssid() {
    return g_ctx.selected_ssid;
}

bool wifi_has_scan_results() {
    return g_ctx.has_scan_results;
}

uint16_t wifi_get_scan_count() {
    return g_ctx.scan_count;
}
