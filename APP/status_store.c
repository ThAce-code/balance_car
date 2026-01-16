#include "status_store.h"

#include "stm32f4xx_hal.h" // __DMB()

typedef struct {
  volatile uint32_t seq;
  StatusData_t data;
} status_store_t;

static status_store_t g_status_store = {0};

void StatusStore_Write(const StatusData_t *s) {
  if (s == NULL) {
    return;
  }

  // seq odd => writer in progress; seq even => stable snapshot
  g_status_store.seq++;
  __DMB();
  g_status_store.data = *s;
  __DMB();
  g_status_store.seq++;
}

bool StatusStore_Read(StatusData_t *out) {
  if (out == NULL) {
    return false;
  }

  // Retry until we observe a stable even seq around the copy.
  while (1) {
    uint32_t s1 = g_status_store.seq;
    if (s1 & 1u) {
      continue;
    }
    __DMB();
    *out = g_status_store.data;
    __DMB();
    uint32_t s2 = g_status_store.seq;
    if (s1 == s2 && ((s2 & 1u) == 0u)) {
      return true;
    }
  }
}

