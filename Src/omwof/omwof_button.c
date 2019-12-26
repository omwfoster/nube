/*
 * omwof_button_menu.c
 *
 *  Created on: 27 Nov 2019
 *      Author: oli
 */

#include <omwof/omwof_button.h>
#include "omwof/omwof_menu.h"
#include "omwof/omwof_window.h"
#include "omwof/omwof_weight.h"



typedef enum button_state {
	NOT_IN_USE, BUTTON_PRESSED, BUTTON_RELEASED
} enum_button_state;
typedef enum button_event {
	NO_EVENT, SHORT_PRESS, LONG_PRESS, EXTENDED_PRESS
} enum_button_event;


enum_button_state user_button = NOT_IN_USE;
enum_button_event user_event = NO_EVENT;
extern menu_typedef * toplevel_menu[];
menu_typedef volatile * active_menu = NULL;

uint8_t init_button() {
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	if (active_menu == NULL) {
		 active_menu = toplevel_menu[0];
	}
}



uint8_t process_event(enum_button_event evt) {

	switch (evt) {
	case SHORT_PRESS:
		active_menu->active_callback = active_menu->active_callback->next_ptr;
			set_window();
		break;
	case LONG_PRESS:
		if(active_menu==toplevel_menu[0]){active_menu = toplevel_menu[1];}
		else{active_menu = toplevel_menu[0];
		}
		break;
	default:
		break;
	}
}


static volatile uint32_t duration = 0;
static volatile  uint32_t start_press = 0U;
static volatile  bool wait_release = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


	volatile uint32_t event = HAL_GetTick();
 	if ((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)&&(start_press == 0)) {
		start_press = event ;
	} else {
		duration = event - start_press;
		user_event = duration < 3000 ? SHORT_PRESS : LONG_PRESS;
		process_event(user_event);
		start_press = 0;
		duration = 0;

	}

}

void EXTI0_IRQHandler(void) {
	/* EXTI line interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_GPIO_EXTI_Callback(KEY_BUTTON_PIN);
	}
}

