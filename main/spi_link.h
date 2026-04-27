#pragma once

#include "dataprocess.h"

bool spi_link_init();
bool spi_link_send_result(const DetectionResult &result);
