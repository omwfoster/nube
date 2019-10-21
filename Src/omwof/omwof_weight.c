/*
 * omwof_weight.c
 *
 *  Created on: 18 Oct 2019
 *      Author: oli
 */


// Provide an attenuation factor for the led output
// this should clamp down when standard deviation is low so as to
// focus the outputs on dominant signal, rather than baseline( high pass filter for frequency domain)

// rms current value
// rms rolling average
// rms peak
// standard deviation
// DC offset




#include <omwof/omwof_weight.h>

float32_t rms_array[5];
float32_t rms_max = 0.0f;
float32_t sd_min = 60000.0f;
float32_t rms_min = 60000.0f;
float32_t weight;
static uint8_t counter = 0;
static float32_t weighting;
static float32_t rolling_avg;


float32_t rms_weighting(float32_t * input_array, uint32_t array_len) {



	arm_rms_f32(input_array, array_len, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	arm_mean_f32(&rms_array[counter], 5, &rolling_avg);
	weight = ((rolling_avg - rms_min) / (rms_max - rms_min));
	(counter < 4) ? ++counter : 0;
	return weight;

}

float32_t sd_weighting(float32_t * input_array, uint32_t array_len) {

	float32_t  pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_len, &pResult);
	arm_rms_f32(input_array, array_len, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	arm_max_f32(rms_array,5,&rms_max,&pIndex);
	arm_mean_f32(&rms_array[counter], 5, &rolling_avg);
//	pResult < sd_min ? pResult : sd_min;
	if (pResult < sd_min){sd_min = pResult;}

	weight = ((rolling_avg - *input_array) / (rms_max - *input_array ));
//	weight = ((rolling_avg - *input_array) / sd_min);
	(counter < 4) ? ++counter : 0;
	return weight;

}

