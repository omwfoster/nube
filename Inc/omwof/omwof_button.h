/*
 * omwof_button_menu.h
 *
 *  Created on: 27 Nov 2019
 *      Author: oli
 */

#ifndef OMWOF_OMWOF_BUTTON_H_
#define OMWOF_OMWOF_BUTTON_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <arm_math.h>
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "gpio_defines.h"
#include "omwof/omwof_window.h"
#include "omwof/omwof_weight.h"

extern Window_TypeDef  Window_profiles[];
uint8_t active_Window_index;
extern Weight_TypeDef Weight_profiles[];
uint8_t active_Weight_index;




#endif /* OMWOF_OMWOF_BUTTON_H_ */
