#ifndef STATUS_STORE_H
#define STATUS_STORE_H

#include <stdbool.h>

#include "mydefine.h"

// Single-writer (MainControlTask) / multi-reader (OLED/Comm/...) latest-status store.
// Uses a simple sequence counter to avoid torn reads without taking a mutex.
void StatusStore_Write(const StatusData_t *s);
bool StatusStore_Read(StatusData_t *out);

#endif /* STATUS_STORE_H */

