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
float32_t sd_max = 0.0f;
float32_t sd_min = 60000.0f;
float32_t rms_min = 60000.0f;
float32_t weight;
static uint8_t counter = 0;
static float32_t weighting;
static float32_t rolling_avg;
static float32_t sd_dev;

// estimate dc component from *input_array[0] bin
float32_t rms_weighting(float32_t * input_array, uint32_t array_len,
		float32_t * st_dev) {

	float32_t pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_len, &sd_dev);

	arm_rms_f32(input_array, array_len, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	weight = (rms_array[counter] - *input_array) / (rms_max - *input_array);
	(counter < 4) ? ++counter : 0;
	*st_dev = 1;
	return weight;

}

// estimate component from rolling minimum rms value

float32_t rms_weighting_2(float32_t * input_array, uint32_t array_len,
		float32_t * st_dev) {

	float32_t pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_len, st_dev);

	arm_rms_f32(input_array, array_len, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	weight = (rms_array[counter] - rms_min) / (rms_max - rms_min);
	(counter < 4) ? ++counter : 0;
	*st_dev = 1;
	return weight;

}

volatile float32_t sd_weight;
float32_t sd_weighting(float32_t * input_array, uint32_t array_len,
		float32_t * st_dev) {

	float32_t pResult;
	uint32_t pIndex;

	arm_std_f32(input_array, array_len, &pResult);

	arm_rms_f32(input_array, array_len, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];

	sd_max = (pResult > sd_max) ? pResult : sd_max;
	sd_min = (pResult < sd_min) ? pResult : sd_min;

	weight = ((rolling_avg - *input_array) / (rms_max - *input_array));

	sd_weight = ((sd_max - sd_min) / (sd_dev - pResult));
	* st_dev = sd_weight;
	return weight;

}

float32_t sd_weighting_2(float32_t * input_array, uint32_t array_len,
		float32_t * st_dev)
	{

		float32_t sd_samples;
		uint32_t pIndex;
		arm_std_f32(input_array, array_len, &sd_samples);
		arm_rms_f32(input_array, array_len, &rms_array[counter]);
		rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
		arm_max_f32(rms_array, 5, &rms_max, &pIndex);
		arm_mean_f32(&rms_array[counter], 5, &rolling_avg);
		if (sd_samples < sd_min) {
			sd_min = sd_samples;
		}
		//weight = (*input_array - rms_array[counter])/ sd_samples;
		(counter < 4) ? ++counter : 0;
		return weight;

	}

