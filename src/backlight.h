#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif 

void backlight_init(void);
void backlight_enable(bool on);

#ifdef __cplusplus
}
#endif