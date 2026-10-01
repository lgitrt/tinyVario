/*
 * button.c — Button debounce and hold detection
 *
 * Author: Luca Obwegs
 */
#include "button.h"

/* Returns 1 if the button is held for exactly 2 seconds, 0 if released early. */
uint8_t Is_Button_Held_2s(void) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= 2000) return 1;
        HAL_Delay(10);
    }
    return 0;
}
