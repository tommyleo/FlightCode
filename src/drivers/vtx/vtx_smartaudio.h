#pragma once
#include <stdbool.h>
bool vtx_smartaudio_init(void);
void vtx_smartaudio_update(bool armed);
const char *vtx_smartaudio_status_name(void);
