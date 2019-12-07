/*
 * omwof_window.h
 *
 *  Created on: 22 Nov 2019
 *      Author: oli
 */


#ifndef OMWOF_OMWOF_WINDOW_H_
#define OMWOF_OMWOF_WINDOW_H_

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>



extern float32_t *hann_ptr;

typedef struct{
	uint8_t (*WindowFunc)(float32_t *, uint32_t);
	uint8_t WindowName[10];
	uint8_t WindowIndex;
}Window_TypeDef;

extern Window_TypeDef  Window_profiles[5];

float32_t Hanning(float32_t *,uint32_t);
float32_t Hamming(float32_t *,uint32_t);
float32_t Chebeyshev(float32_t *,uint32_t);
float32_t Blackman(float32_t *,uint32_t);
float32_t Kaiser(float32_t *,uint32_t);



#define COUNT_OF_WINDOWS(x)  (sizeof(x)/sizeof(Window_TypeDef))



#endif /* OMWOF_OMWOF_WINDOW_H_ */
