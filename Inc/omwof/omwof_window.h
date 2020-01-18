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
	void (*WindowFunc)(float32_t *, uint32_t);
	uint8_t WindowName[10];
	uint8_t WindowIndex;
}Window_TypeDef;

extern Window_TypeDef  Window_profiles[5];

void Hanning(float32_t *,uint32_t);
void Hamming(float32_t *,uint32_t);
void Chebeyshev(float32_t *,uint32_t);
void Blackman(float32_t *,uint32_t);
void Kaiser(float32_t *,uint32_t);



#define COUNT_OF_WINDOWS(x)  (sizeof(x)/sizeof(Window_TypeDef))



#endif /* OMWOF_OMWOF_WINDOW_H_ */
