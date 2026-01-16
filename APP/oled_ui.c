#include "oled_ui.h"

#include "OLED/ssd1306.h"
#include "OLED/ssd1306_fonts.h"

#include <math.h>
#include <stdio.h>

static uint8_t s_oled_inited = 0;

static void format_pitch(char *out, size_t out_len, float pitch_deg) {
  // Avoid float printf in newlib-nano for stability/size; format as signed centi-degrees.
  int32_t cdeg = (int32_t)lroundf(pitch_deg * 100.0f);
  char sign = '+';
  if (cdeg < 0) {
    sign = '-';
    cdeg = -cdeg;
  }
  int32_t deg = cdeg / 100;
  int32_t frac = cdeg % 100;
  (void)snprintf(out, out_len, "PITCH:%c%ld.%02lddeg", sign, (long)deg, (long)frac);
}

static void format_speed(char *out, size_t out_len, const char *label, float speed_mps) {
  // Signed centi-(m/s)
  int32_t cms = (int32_t)lroundf(speed_mps * 100.0f);
  char sign = '+';
  if (cms < 0) {
    sign = '-';
    cms = -cms;
  }
  int32_t mps_i = cms / 100;
  int32_t frac = cms % 100;
  (void)snprintf(out, out_len, "%s:%c%ld.%02ld", label, sign, (long)mps_i, (long)frac);
}

void OledUi_Init(void) {
  if (s_oled_inited) {
    return;
  }

  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  (void)ssd1306_WriteString("BALANCE CAR", Font_7x10, White);
  ssd1306_UpdateScreen();

  s_oled_inited = 1;
}

void OledUi_Render(const StatusData_t *s) {
  if (!s_oled_inited) {
    OledUi_Init();
  }

  char line1[24];
  char line2[24];
  char line3[24];

  format_pitch(line1, sizeof(line1), s->pitch_deg);
  char lbuf[12];
  char rbuf[12];
  format_speed(lbuf, sizeof(lbuf), "L", s->wheel_l_mps);
  format_speed(rbuf, sizeof(rbuf), "R", s->wheel_r_mps);
  (void)snprintf(line2, sizeof(line2), "%s %s", lbuf, rbuf);
  (void)snprintf(line3, sizeof(line3), "M:%u F:%04X", (unsigned)s->mode, (unsigned)s->fault_bits);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  (void)ssd1306_WriteString(line1, Font_7x10, White);
  ssd1306_SetCursor(0, 16);
  (void)ssd1306_WriteString(line2, Font_7x10, White);
  ssd1306_SetCursor(0, 32);
  (void)ssd1306_WriteString(line3, Font_7x10, White);
  ssd1306_UpdateScreen();
}
