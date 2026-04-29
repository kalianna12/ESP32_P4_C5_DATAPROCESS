#pragma once

#include "dataprocess.h"

bool spi_link_init();
bool spi_link_send_result(const DetectionResult &result);
bool spi_link_send_test_frequency(uint16_t detected_hz);
