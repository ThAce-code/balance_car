#ifndef OLED_UI_H
#define OLED_UI_H

#include <stdint.h>

#include "mydefine.h"

// OLED UI init: call once from OLEDDisplayTask before first render.
void OledUi_Init(void);

// Render one frame (pitch + mode/fault). Call only from OLEDDisplayTask.
void OledUi_Render(const StatusData_t *s);

#endif /* OLED_UI_H */

