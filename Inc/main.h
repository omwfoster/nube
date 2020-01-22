
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
#include "omwof/omwof_weight.h"
#include "omwof/omwof_irq.h"
#include "omwof/omwof_lcd.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

#define I2C1_OPEN
//#define I2C2_OPEN

#define I2C_SCL_SPEED       (200000)    // SCL in Hz
#define I2C_OWN_ADDR        (0x00)      // use 0x00 for master

/**
 * @brief Definition for I2C1
 */
#if defined	I2C1_OPEN
	#define Open_I2C                        I2C1
	#define Open_I2C_CLK                    RCC_APB1Periph_I2C1

	#define Open_I2C_SCL_PIN                GPIO_Pin_6
	#define Open_I2C_SCL_GPIO_PORT          GPIOB
	#define Open_I2C_SCL_GPIO_CLK           RCC_AHB1Periph_GPIOB
	#define Open_I2C_SCL_SOURCE             GPIO_PinSource6
	#define Open_I2C_SCL_AF                 GPIO_AF_I2C1

	#define Open_I2C_SDA_PIN                GPIO_Pin_9
	#define Open_I2C_SDA_GPIO_PORT          GPIOB
	#define Open_I2C_SDA_GPIO_CLK           RCC_AHB1Periph_GPIOB
	#define Open_I2C_SDA_SOURCE             GPIO_PinSource9
	#define Open_I2C_SDA_AF                 GPIO_AF_I2C1

	#define Open_I2C_IRQn                   I2C1_EV_IRQn
	#define I2Cx_IRQHANDLER                 I2C1_EV_IRQHandler

#elif defined I2C2_OPEN

#endif





#define SAMPLE_RATE_HZ  16000U // Sample rate of the audio in hertz.
#define FFT_LEN 1024
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

// Todo: test  functionality
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
#define INTERNAL_BUFF_SIZE                    128*DEFAULT_AUDIO_IN_FREQ/8000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000




void enablefpu(void);


typedef enum
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;












/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24) *  * ------------------------------------------------------------------- */
//extern float32_t *Hanning(float32_t *,uint32_t, uint8_t);
extern float32_t *hann_ptr;
float32_t *avg_weighting();
float32_t *rolling_avg();



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
void set_window();
void cleanbuffers();







/* USER CODE END Private defines */


void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */


