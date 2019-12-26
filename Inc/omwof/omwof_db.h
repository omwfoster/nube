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

#endif /* OMWOF_OMWOF_DB_H_ */
