#include "led.h"



void led_task(void) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}