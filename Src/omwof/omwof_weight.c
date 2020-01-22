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

float32_t sd_max = 0.0f;
float32_t sd_min = 60000.0f;

volatile float32_t weight;
static float32_t rolling_avg;
static float32_t sd_dev;

float32_t db_max;
float32_t db_min;
float32_t db_mean;
float32_t db_running_mean;

Weight_TypeDef Weight_profiles[] = { { rms_weighting, "rms_1", 0 }, {
		rms_weighting_2, "rms_1", 2 }, { sd_weighting, "sd__1", 1 }, {
		sd_weighting_2, "sd__2", 3 }, };



// estimate dc component from *input_array[0] bin
float32_t rms_weighting(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	static volatile uint32_t counter = 0;
	static float32_t rolling_avg = 0;
	static volatile float32_t rms_max = 0.0f;
	static volatile float32_t rms_min = 60000.0f;

	arm_rms_f32(input_array, array_length, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	arm_mean_f32(&rms_array[0], 5, &rolling_avg);
	weight = (rolling_avg - rms_min) / (rms_max - rms_min);
	(counter < 4) ? ++counter : 0;
	*st_dev = 1;
	return weight;

}

// estimate component from rolling minimum rms value

float32_t rms_weighting_2(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	static volatile uint32_t counter = 0;
	static float32_t rolling_avg = 0;
	static volatile float32_t rms_max = 0.0f;
	static volatile float32_t rms_min = 60000.0f;

	arm_rms_f32(input_array, array_length, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	weight = (rms_array[counter] - rms_min) / (rms_max - rms_min);
	(counter < 4) ? ++counter : 0;
	*st_dev = 1;
	return weight;

}

float32_t sd_weighting(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	static volatile uint32_t counter = 0;
	static float32_t rolling_avg = 0;
	static volatile float32_t rms_max = 0.0f;
	static volatile float32_t rms_min = 60000.0f;

	float32_t sd_weight;
	float32_t pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_length, &pResult);
	arm_rms_f32(input_array, array_length, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	arm_mean_f32(&rms_array[0], 5, &rolling_avg);
	weight = (pResult - (rolling_avg) / (rms_max - pResult));
	sd_weight = ((sd_max - sd_min) / (sd_dev - pResult));
	*st_dev = sd_weight;
	(counter < 4) ? ++counter : 0;
	return weight;

}

float32_t sd_weighting_2(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	static volatile uint32_t counter = 0;
	static float32_t rolling_avg = 0;
	static volatile float32_t rms_max = 0.0f;
	static volatile float32_t rms_min = 60000.0f;
	float32_t sd_weight;
	float32_t pResult;
	float32_t sd_samples;
	uint32_t pIndex;
	//remove_dc_from_mag(input_array, array_length);
	arm_std_f32(input_array, array_length, &sd_samples);
	arm_rms_f32(input_array, array_length, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	sd_max = (sd_samples > sd_max) ? sd_samples : sd_max;
	sd_min = (sd_samples < sd_min) ? sd_samples : sd_min;
	weight = ((rolling_avg - *input_array) / (rms_max - *input_array));
	sd_weight = ((sd_max - sd_min) / (sd_dev - pResult));
	*st_dev = sd_weight;
	(counter < 4) ? ++counter : 0;
	return weight;

}

