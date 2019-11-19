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
static float32_t rolling_avg;
static float32_t sd_dev;

float32_t db_max;
float32_t db_min;
float32_t db_mean;
float32_t db_running_mean;

void clean_weight() {

	rms_max = 0.0f;
	sd_max = 0.0f;
	sd_min = 60000.0f;
	rms_min = 60000.0f;
}
void remove_dc_from_mag(float32_t * mag, uint32_t array_length) {

	float32_t t_float[array_length];
	float32_t dc_component = *mag;
	float32_t * mag_bin = mag;

	for (uint32_t i = 0; i < ((array_length / 2) - 1); ++i) {
		mag_bin += 2;
		*mag_bin = *mag_bin - dc_component;
	}
}

void remove_rms_from_wave(float32_t * samples, uint32_t array_length) {
	float32_t temp_mean = 0.0f;

	float32_t t_float[array_length];

	arm_mean_f32(samples, array_length, &temp_mean);
	arm_offset_f32(samples, ((temp_mean * -1) / array_length), &t_float[0],
			array_length); // multiply by -1 for -offset
	memcpy(samples, &t_float[0], array_length * 4);

}

volatile float32_t dynamic_range_max = 130.0f;
volatile float32_t dynamic_range_min = 63555.0f;

void mag2db(float32_t * array, float32_t * db_array, uint32_t array_len) {

	static float32_t t_avg;
	static volatile float32_t t, u;
	static volatile float32_t arm_max_result = 0.0f;
	static float32_t arm_min_result = 63555.0f;

	arm_abs_f32(array, db_array, array_len);

	for (uint32_t i = 0; i < array_len; ++i) {
		if (*array > 0.0f) {
			*db_array = 20 * log10(*array);
			++array;
			++db_array;
		} else {
			*db_array = -99.999f;
			++array;
			++db_array;
		}
	}

}

float32_t shift_dB_array[64];

void shift_db_to_100(float32_t * db_array, uint32_t array_len) {
	static float32_t t_max;
	static volatile float32_t t_offset;
	static uint32_t i, temp_db;

	arm_max_f32(db_array, array_len, &t_max, &i);
	arm_offset_f32(db_array, (t_max * -1), shift_dB_array, array_len);

	for (uint32_t i = 0; i < array_len; i++) {
		shift_dB_array[i] =
				shift_dB_array[i] > -100 ? shift_dB_array[i] : -99.99f;

	}

	arm_offset_f32(shift_dB_array, 100, db_array, array_len);

}

void normalize_db(float32_t * dB_array, uint32_t array_len) {
	static float32_t min;
	uint32_t * pIndex;
	arm_scale_f32(dB_array, 0.01f, &shift_dB_array[0], array_len);
	arm_min_f32(shift_dB_array, array_len, &min, &pIndex);
	arm_offset_f32(shift_dB_array, (-1.0*min), dB_array, array_len);
	arm_scale_f32(dB_array,(1/(1.0f - min)),shift_dB_array,array_len);
	memcpy(dB_array,shift_dB_array,array_len *4);



}

// estimate dc component from *input_array[0] bin
float32_t rms_weighting(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	float32_t pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_length, &sd_dev);

	arm_rms_f32(input_array, array_length, &rms_array[counter]);
	rms_max = (rms_max > rms_array[counter]) ? rms_max : rms_array[counter];
	rms_min = (rms_min < rms_array[counter]) ? rms_min : rms_array[counter];
	weight = (rms_array[counter] - *input_array) / (rms_max - *input_array);
	(counter < 4) ? ++counter : 0;
	*st_dev = 1;
	return weight;

}

// estimate component from rolling minimum rms value

float32_t rms_weighting_2(float32_t * input_array, uint32_t array_length,
		float32_t * st_dev) {

	float32_t pResult;
	uint32_t pIndex;
	arm_std_f32(input_array, array_length, st_dev);

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

