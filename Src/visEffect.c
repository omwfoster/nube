#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "../Src/ws2812b/ws2812b.h"
#include <stdlib.h>
#include <ws2812b.h>
#include <arm_math.h>
#include "visEffect.h"

uint8_t w_pos;

extern WS2812_BufferItem * ws2812b_getBufferItem(ws_buf_state status);

rgb HSV2RGB(hsv in) {
	float32_t hh, p, q, t, ff;
	uint16_t i;
	rgb out;

	if (in.s <= 0.0) {
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}
	hh = in.h;
	if (hh >= 360.0)
		hh = 0.0;
	hh /= 60.0;
	i = (uint16_t) hh;
	ff = hh - i;
	p = in.v * (1.0f - in.s);
	q = in.v * (1.0f - (in.s * ff));
	t = in.v * (1.0f - (in.s * (1.0 - ff)));

	switch (i) {
	case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	case 5:
	default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	}
	return out;
}

void visInit() {

	ws2812b_init();
}



// Process FFT and populate idle buffer

uint8_t generate_RGB(float32_t * fft, float32_t * mag, uint32_t array_len) {


	static WS2812_BufferItem * ws_item_ptr;
	static float32_t f32_FFT_len = (float32_t) FFT_LEN;
	static ws_buf_state bs = WS_NOT_IN_USE;
	float32_t * _Real;
	float32_t * _mag = mag;
	hsv hsv_struct;
	volatile rgb rgb_struct;
	static uint8_t volatile * u_ptr;
	//ws_item_ptr = Null;
	ws_item_ptr = ws2812b_getBufferItem(bs);
	//ws_item_ptr = (WS2812_BufferItem *) ws2812b_getBufferItem(TEST); // TEST CODE


	if (ws_item_ptr != NULL) {
		ws_item_ptr->WS2812_buf_state = WS_WRITE_LOCKED;
		u_ptr = ws_item_ptr->rgb_Buffer_ptr;
		_Real = fft;
		for (uint16_t i = 1; i < (FFT_LEN / 2); ++i) { ///  hard-coded buffer size need runtime evaluation

			if (*(_mag) > 0.0f) {
				hsv_struct.h = (atan(*(_mag) / *(_Real))) * (180.0 / PI);
				hsv_struct.s = (*(_mag) / *(_Real));
				hsv_struct.v = ((*_Real) / f32_FFT_len);
				_Real += 2;
				_mag++;
				rgb_struct = HSV2RGB(hsv_struct);

				*u_ptr = (uint8_t) (rgb_struct.r * 255);
				++u_ptr;
				*u_ptr = (uint8_t) (rgb_struct.g * 255);
				++u_ptr;
				*u_ptr = (uint8_t) (rgb_struct.b * 255);
				++u_ptr;

			} else {
				_Real += 2;
				_mag++;
				*u_ptr = 0;
				++u_ptr;
				*u_ptr = 0;
				++u_ptr;
				*u_ptr = 0;
				++u_ptr;
			}

		}

		//	ws_item_ptr->WS2812_buf_state = BUFFER_FULL;
		ws_item_ptr->WS2812_buf_state = WS_BUFFER_FULL;
		ws_item_ptr = NULL;

		return 1;
	} else {
		return 0;
	}

}

uint8_t generate_BB() {

	// Transfer data from the processed FFT to
	// Transfer to the Bit Buffer Area

	static ws_buf_state bs = WS_BUFFER_FULL;
	static volatile WS2812_BufferItem * bf;
	bf = (WS2812_BufferItem *) ws2812b_getBufferItem(bs);
	if (bf != NULL) {
		BB_generator(bf);
		bf = NULL;
		return 1;
	}
	bf = NULL;
	return 0 ;

}

#ifdef __cplusplus
}
#endif

