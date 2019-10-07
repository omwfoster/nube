/*
 * omwof_test.c
 *
 *  Created on: 4 Sep 2019
 *      Author: oliver foster
 */

#include "omwof/omwof_test.h"

uint8_t init_chunk(uint32_t data_rate, uint32_t sample_freq, chunk_TypeDef * ts) {

	static uint8_t samples_per_cycle;
	float32_t * ptr_impulse;
	ts->chunk_len = data_rate / sample_freq;
	ts->ptr_array = malloc((ts->chunk_len) * 4);
	memset(ts->ptr_array, 0, (ts->chunk_len) * 4);
	return 1;

}

void sine_chunk(chunk_TypeDef * ts) {

	volatile float32_t * ptr_impulse = ts->ptr_array;
	uint8_t i = ts->chunk_len;

	while (i > 0) {
		*ptr_impulse = sin((2 * PI) * i / (ts->chunk_len));
		ptr_impulse++;
		i--;
	}

}

void copy_chunk(float32_t * output, uint32_t len, chunk_TypeDef * ts) {
	float32_t * d_loc = output;
	uint8_t repeats = len / ts->chunk_len;
	for (uint8_t i = 0; i < repeats; ++i) {
		memcpy(d_loc, ts->ptr_array, (ts->chunk_len * 4));
		d_loc += ts->chunk_len;

	}
	free(ts->ptr_array);

}

void sine_sample(float32_t * output, uint32_t array_len, uint32_t cycle_len) {

	float32_t volatile * next = output;

	volatile float32_t k = M_TWOPI / cycle_len;

	for (uint8_t j = 0; j < (cycle_len); ++j)
	{
		*next = arm_sin_f32(k * j);
		next++;
	}


	next = output;
	for (uint8_t cycles = 0; cycles < (array_len / cycle_len); ++cycles) {
		memcpy(next,output,(cycle_len * 4));
		next += cycle_len;
	}

}

