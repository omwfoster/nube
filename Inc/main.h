
#ifndef __MAIN_H__
#define __MAIN_H__

//#define TEST

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "gpio_defines.h"
#include "omwof/omwof_test.h"





#define SAMPLE_RATE_HZ  16000U // Sample rate of the audio in hertz.
#define FFT_LEN 128
#define FFT_BUFFER_SIZE  (FFT_LEN * 2) /* size in bytes of the fft input buffer */
#define NUM_BLOCKS  (FFT_BUFFER_SIZE / BLOCK_SIZE)


/* USER CODE BEGIN Private defines */


/* Defines for LEDs lighting */
#define LED3_TOGGLE 0x03 /* Toggle LED3 */
#define LED4_TOGGLE 0x04 /* Toggle LED4 */
#define LED6_TOGGLE 0x06 /* Toggle LED6 */
#define LEDS_OFF 0x07    /* Turn OFF all LEDs */
#define STOP_TOGGLE 0x00 /* Stop LED Toggling */

/* Defines for MEMS Acclerometer ID Types */
#define MEMS_LIS3DSH 0x3F  /* LIS3DSH MEMS Acclerometer ID */
#define MEMS_LIS302DL 0x3B /* LIS302DL MEMS Acclerometer ID */

// Todo: test  fuctionality
#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */

/* Audio status definition */
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */

#define DEFAULT_AUDIO_IN_FREQ                 16000U
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */


/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000









void enablefpu(void);


typedef enum
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;








/* FFT -------------------------------------------------------------*/
// todo : Problem with hardfault caused by imprecise data






/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24) *  * ------------------------------------------------------------------- */
float32_t *Hanning(uint32_t, uint8_t);
float32_t *avg_weighting();
float32_t *rolling_avg();
float32_t *hann_ptr;


void PCM_to_Float(uint16_t *, float32_t *, uint16_t);
void StartFIRTask();
uint8_t StartRFFTTask();
void enablefpu();
void clean_input_buffers();
void clean_output_buffers();
int timer_setup();
void fft_ws2812_Init();
void Error_Handler(void);
void calc_mag_output(float32_t *, float32_t *, uint16_t);
void drop_volume();







/* USER CODE END Private defines */


void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */


