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

rgb hsv2rgb(hsv in) {
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

	i_test = 0;

	memset(frame_Buffer, 0, sizeof(frame_Buffer));
	memset(frame_Buffer2, 0, sizeof(frame_Buffer2));
	ws2812b_init(&frame_Buffer[0], &frame_Buffer2[0], (3 * (FFT_LEN / 2)));
	ws2812b.transferComplete = 1;

}

uint8_t visHandle() {

	ws2812b_handle();

	return 0;

}

uint8_t generate_rgb(float32_t * fft, float32_t * mag, uint32_t array_len) {

	static float32_t f32_FFT_len = (float32_t) FFT_LEN;
	float32_t * _Real;
	float32_t * _mag = mag;
	hsv hsv_struct;
	volatile rgb rgb_struct;
	static volatile uint8_t * u_ptr;
	static WS2812_BufferItem * ws_item_ptr;
	uint8_t  * buf_ptr = NULL;
	* buf_ptr = ws2812b_getBufferItem();

	if (ws_item_ptr != NULL) {
		ws_item_ptr->buf_state = WRITE_LOCKED;
		u_ptr = ws_item_ptr->frameBufferPointer;
		_Real = fft;
		for (uint16_t i = 1; i < (sizeof(frame_Buffer) / 3); ++i) {

			if (*(_mag) > 0.0f) {
				hsv_struct.h = (atan(*(_mag) / *(_Real))) * (180.0 / PI);

				hsv_struct.v = (*(_mag) / *(_Real));
				hsv_struct.s = ((*_Real) / f32_FFT_len);
				_Real += 2;
				_mag++;
				rgb_struct = hsv2rgb(hsv_struct);

				//*(++u_ptr) = (uint8_t) (rgb_struct.r * 255);
				//*(++u_ptr) = (uint8_t) (rgb_struct.g * 255);
				//*(++u_ptr) = (uint8_t) (rgb_struct.b * 255);
				*(++u_ptr) = 80;
				*(++u_ptr) = 80;
				*(++u_ptr) = 80;

			} else {
				_Real += 2;
				_mag++;
				*(++u_ptr) = 0;
				*(++u_ptr) = 0;
				*(++u_ptr) = 0;
			}

		}

		ws2812b.transferComplete = 0;
		ws2812b.startTransfer = 1;
		ws_item_ptr->buf_state = BUFFER_FULL;
		u_ptr = NULL;
		ws_item_ptr = NULL;
		return 1;
	} else {
		return 0;
	}

}

#ifdef __cplusplus
}
#endif

