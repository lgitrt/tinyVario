/*
 * button.h — Button debounce and hold detection
 *
 * Author: Luca Obwegs
 */
#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

#define BUTTON_PORT  ButtonIn_GPIO_Port
#define BUTTON_PIN   ButtonIn_Pin
#define USB_PORT     USBsens_GPIO_Port
#define USB_PIN      USBsens_Pin

uint8_t Is_Button_Held_2s(void);

#endif /* INC_BUTTON_H_ */
