
#ifndef VISEFFECT_H_
#define VISEFFECT_H_



#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Includes */


#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>
#include "../Src/ws2812b/ws2812b.h"



extern WS2812_BufferItem * getBufferItem(uint8_t status);
extern uint8_t frame_Buffer[];
extern uint8_t frame_Buffer2[];
uint8_t generate_BB();



// Helper defines
#define newColor(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) <<  8) | (b))
#define Red(c) ((uint8_t)((c >> 16) & 0xFF))
#define Green(c) ((uint8_t)((c >> 8) & 0xFF))
#define Blue(c) ((uint8_t)(c & 0xFF))

uint16_t s;
void visInit();
void visHandle();
uint8_t setBuffer_BaseAddress(uint8_t,uint8_t *,uint32_t);  //give base address  (buffer number,base address,buffer length)
void visInit();
uint8_t generate_RGB(float32_t *,float32_t * ,uint32_t,float32_t);



typedef struct {
	float32_t r;// a fraction between 0 and 1
	float32_t g;// a fraction between 0 and 1
	float32_t b;// a fraction between 0 and 1
}rgb;

typedef struct {
	float32_t h;// angle in degrees
	float32_t s;// a fraction between 0 and 1
	float32_t v;// a fraction between 0 and 1
}hsv;





#ifdef __cplusplus
}
#endif


#endif /* VISEFFECT_H_ */



