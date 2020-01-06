/*
 * omwof_db.h
 *
 *  Created on: 18 Dec 2019
 *      Author: oli
 */

#ifndef OMWOF_OMWOF_DB_H_
#define OMWOF_OMWOF_DB_H_

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>


void remove_dec_from_mag(float32_t *, uint32_t);
void remove_rms_from_wave(float32_t *, uint32_t);
void shift_db_to_100(float32_t * db_array, uint32_t array_len);
void mag2db(float32_t *, float32_t *, uint32_t);
void normalize_db(float32_t *, uint32_t);
void power_spectra(float32_t * input_bins, float32_t * power_spectra,
		uint32_t number_of_bins);
void power_spectra1(float32_t * input_bins, float32_t * power_spectra,
		uint32_t number_of_bins);
void power_spectra2(float32_t * input_bins, float32_t * power_spectra,
		uint32_t number_of_bins);
void power_spectra3(float32_t * input_bins, float32_t * power_spectra,
		uint32_t number_of_bins);

#endif /* OMWOF_OMWOF_DB_H_ */
