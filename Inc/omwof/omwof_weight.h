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



float32_t rms_weighting(float32_t * input_array, uint32_t,float32_t *);
float32_t rms_weighting_2(float32_t * input_array, uint32_t,float32_t *);
float32_t sd_weighting(float32_t * input_array, uint32_t,float32_t *);
float32_t sd_weighting_2(float32_t * input_array, uint32_t,float32_t *);



#endif /* OMWOF_OMWOF_WEIGHT_H_ */
