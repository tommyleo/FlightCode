#pragma once

#include <stdbool.h>

#include "sbus.h"

void osd_tuning_menu_init(void);
void osd_tuning_menu_update(const sbus_data_t *receiver, bool armed);
bool osd_tuning_menu_is_active(void);

