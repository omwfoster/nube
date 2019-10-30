/*
 * omwof_test.h
 *
 *  Created on: 4 Sep 2019
 *      Author: oliver foster
 */

#ifndef OMWOF_TEST_H_
#define OMWOF_TEST_H_


#include "arm_math.h"
#include "stdint.h"
#include "arm_math.h"
#include "stdlib.h"


typedef struct testsample_TypeDef {
	float32_t * ptr_array;
	uint32_t chunk_len;
	uint32_t frequency;
} chunk_TypeDef;


uint8_t init_chunk(uint32_t,uint32_t,chunk_TypeDef *);
void sine_chunk(chunk_TypeDef *);
void copy_chunk(float32_t * , uint32_t, chunk_TypeDef *);
void sine_sample(float32_t * , uint32_t,uint32_t,uint32_t);





#endif /* OMWOF_TEST_H_ */
