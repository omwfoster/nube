/*
 * omwof_weight.h
 *
 *  Created on: 18 Oct 2019
 *      Author: oli
 */

#ifndef OMWOF_OMWOF_WEIGHT_H_
#define OMWOF_OMWOF_WEIGHT_H_

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>


typedef struct{
	float32_t (*WeightFunc)(float32_t *, uint32_t,float32_t *);
	uint8_t WeightName[5];
	uint8_t WeightIndex;
}Weight_TypeDef;
#define COUNT_OF_WEIGHTINGS(x)  (sizeof(x)/sizeof(Weight_TypeDef))
#define STRUCT_COUNT(x,struct_typedef) (sizeof(x)/sizeof(struct_typedef))
extern Weight_TypeDef Weight_profiles[5];



void clean_weight();

float32_t rms_weighting(float32_t * input_array, uint32_t,float32_t *);
float32_t rms_weighting_2(float32_t * input_array, uint32_t,float32_t *);
float32_t sd_weighting(float32_t * input_array, uint32_t,float32_t *);
float32_t sd_weighting_2(float32_t * input_array, uint32_t,float32_t *);



#endif /* OMWOF_OMWOF_WEIGHT_H_ */
