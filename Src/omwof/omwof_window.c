/*
 * omwof_window.c
 *
 *  Created on: 22 Nov 2019
 *      Author: oli
 */

#include <omwof/omwof_window.h>

float32_t Hanning(float32_t * output_array, uint32_t N) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, output_array, N);

	for (i = 0; i < N; i++) { //CALC_HANNING   Calculates Hanning window samples.
		*output_array = 0.5 - (0.5 * (1 - cos(2 * PI * (i + 1) / (N + 1))));
		++output_array;
	}

}

float32_t Hamming(float32_t * output_array, uint32_t N) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, output_array, N);

	for (i = 0; i < N; i++) { //CALC_HANNING   Calculates Hanning window samples.
		*output_array = 0.54 - 0.46 * (1 - cos(2 * PI * (i + 1) / (N + 1)));
		++output_array;
	}

}

float32_t Blackman(float32_t * output_array, uint32_t N) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, output_array, N);

	for (i = 0; i < N; i++) { //CALC_HANNING   Calculates Hanning window samples.
		*output_array = 0.42 - 0.5 * (1 - cos(2 * PI * (i + 1) / (N + 1)))
				+ 0.08 * (1 - cos(2 * PI * (i + 1) * 2 / (N + 1)));
		++output_array;
	}

}

float32_t Kaiser(float32_t * output_array, uint32_t N) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, output_array, N);

	for (i = 0; i < N; i++) { //CALC_HANNING   Calculates Hanning window samples.
		*output_array = 0.5 * (1 - cos(2 * PI * (i + 1) / (N + 1)));
		++output_array;
	}

}

float32_t Chebeyshev(float32_t * output_array, uint32_t N) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, output_array, N);

	for (i = 0; i < N; i++) { //CALC_HANNING   Calculates Hanning window samples.
		*output_array = 0.5 * (1 - cos(2 * PI * (i + 1) / (N + 1)));
		++output_array;
	}

}

Window_TypeDef Window_profiles[5] = { { &Hanning, "Hanning", 0U }, { &Hamming,
		"Hamming", 1U }, { &Blackman, "Blackman", 2U }, { Chebeyshev,
		"Chebeyshev", 3U }, { &Kaiser, "Kaiser", 4U }, };

uint8_t window_init() {



}

