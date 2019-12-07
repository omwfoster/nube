/*
 * omwof_button_menu.c
 *
 *  Created on: 27 Nov 2019
 *      Author: oli
 */

#include <omwof/omwof_button.h>
#include "omwof/omwof_window.h"
#include "omwof/omwof_weight.h"

static uint32_t start_press = 0U;
static bool wait_release = false;

typedef enum button_state {
	NOT_IN_USE, BUTTON_PRESSED, BUTTON_RELEASED
} enum_button_state;
typedef enum button_event {
	NO_EVENT, SHORT_PRESS, LONG_PRESS, EXTENDED_PRESS
} enum_button_event;
enum_button_state user_button = NOT_IN_USE;
enum_button_event user_event = NO_EVENT;

uint8_t process_event(enum_button_event evt) {

	switch (evt) {
	case SHORT_PRESS:
		break;
	case LONG_PRESS:
		break;
	case EXTENDED_PRESS:
		break;
	default:
		break;
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	uint32_t duration = 0;
	uint8_t rising = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if (rising) {
		start_press = HAL_GetTick();
	} else {
		duration = HAL_GetTick() - start_press;
		user_event = duration > 100 ? SHORT_PRESS : LONG_PRESS;
		process_event(user_event);

	}

}

void EXTI0_IRQHandler(void) {
	/* EXTI line interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_GPIO_EXTI_Callback(KEY_BUTTON_PIN);
	}
}

