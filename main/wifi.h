#pragma once

#include <cstddef>
#include <cstdint>

void wifi_init_storage();
bool wifi_ensure_stack_ready();
bool wifi_is_connected_with_ip();
void wifi_start_scan();
void wifi_print_status();
void wifi_autoconnect();
void wifi_disconnect_current();

bool wifi_select_ap_by_index(int index);
void wifi_connect_selected_ap(const char *password);
const char *wifi_get_selected_ssid();
bool wifi_has_scan_results();
uint16_t wifi_get_scan_count();
