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
	TIM5_config();
	HAL_NVIC_EnableIRQ(KEY_BUTTON_EXTI_IRQn);

	if (active_menu == NULL) {
		active_menu = toplevel_menu[0];
	}
}

TIM_HandleTypeDef TIM_Handle_btn_delay;

uint8_t TIM5_config(void)

{
	__TIM5_CLK_ENABLE()
	;
	TIM_Handle_btn_delay.Init.Prescaler = 100;
	TIM_Handle_btn_delay.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle_btn_delay.Init.Period = 16000;
	TIM_Handle_btn_delay.Instance = TIM5;   //Same timer whose clocks we enabled
	return 1;
}


uint8_t process_event(enum_button_event evt) {

	static uint8_t i = 0;
	switch (evt) {
	case SHORT_PRESS:
		active_menu->active_callback = active_menu->active_callback->next_ptr;
		set_window();
		break;
	case LONG_PRESS:
		if (active_menu != toplevel_menu[2]) {
			i++;
		} else {
			i = 0;
		}
		active_menu = toplevel_menu[i];
		break;
	default:
		break;
	}
	HAL_TIM_Base_Stop_IT(&TIM_Handle_btn_delay); // start timer interrupts
	__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
	HAL_NVIC_EnableIRQ(KEY_BUTTON_EXTI_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	volatile uint32_t event = HAL_GetTick();
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {

		HAL_TIM_Base_Init(&TIM_Handle_btn_delay);     // Init timer
		HAL_TIM_Base_Start_IT(&TIM_Handle_btn_delay); // start timer interrupts
		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);

	}

}

void EXTI0_IRQHandler(void) {
	/* EXTI line interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_NVIC_DisableIRQ(KEY_BUTTON_EXTI_IRQn);
		HAL_GPIO_EXTI_Callback(KEY_BUTTON_PIN);
	}
}

uint8_t button_tick = 0;

void TIM5_IRQHandler(void)

{

	volatile enum_button_event evt;

	if (__HAL_TIM_GET_FLAG(&TIM_Handle_btn_delay, TIM_FLAG_UPDATE) != RESET) //In case other interrupts are also running
			{
		if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle_btn_delay, TIM_IT_UPDATE)
				!= RESET) {
			__HAL_TIM_CLEAR_FLAG(&TIM_Handle_btn_delay, TIM_FLAG_UPDATE);

			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
				button_tick++;
			} else {
				evt = (button_tick < 3) ? SHORT_PRESS : LONG_PRESS;
				process_event(evt);
				button_tick = 0;
			}

		}
	}
}

