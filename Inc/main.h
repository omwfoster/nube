/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__


#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"



#define SAMPLE_RATE_HZ  16000U // Sample rate of the audio in hertz.
#define FFT_LEN 128
#define FFT_BUFFER_SIZE  (FFT_LEN) /* size in bytes of the fft input buffer */
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


#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */

/* Audio status definition */
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */

#define DEFAULT_AUDIO_IN_FREQ                 16000U
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME               32

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000




/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */



uint16_t InternalBuffer[INTERNAL_BUFF_SIZE]; // read raw pdm input    128 * DEFAULT_AUDIO_IN_FREQ/16000 *DEFAULT_AUDIO_IN_CHANNEL_NBR
uint16_t PCM_Buf[PCM_OUT_SIZE]; //PCM stereo samples are saved in RecBuf  DEFAULT_AUDIO_IN_FREQ/1000
float32_t float_array[PCM_OUT_SIZE];
static const uint32_t sample_runs = 15;

float32_t FFT_Input[FFT_LEN]; //
float32_t FFT_Bins[FFT_LEN];
float32_t FFT_MagBuf[FFT_LEN / 2]; //
float32_t FFT_MagBuf_IIR[FFT_LEN / 2]; //
float32_t BQC_Buf[FFT_LEN];
/* Width of the peak */





extern void SystemClock_Config(void);
void enablefpu(void);


typedef enum
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;


arm_rfft_fast_instance_f32 rfft_s;







typedef struct {
	union {
		float32_t float_magnitude[FFT_LEN];
		char bytes_magnitude[4*FFT_LEN];
	};
	char * eol;
}output_buffer;

/* Private define ------------------------------------------------------------*/



/* FFT -------------------------------------------------------------*/

extern float32_t FFT_Input[FFT_LEN];//
extern float32_t FFT_Bins[FFT_LEN];
extern float32_t FFT_MagBuf[FFT_LEN/2];//
extern float32_t FFT_MagBuf_IIR[FFT_LEN/2];//
extern float32_t FIR_Buf[FFT_LEN];






typedef enum
{
	RFFT,
	FFT,
	FIR,
	BIQUAD
}filter_typedef;






/* Two filter instances so we can use one while calculating the coefficients of the other */
//static const arm_biquad_casd_df1_inst_f32 peq1_instanceA = {1, peq1_state, peq1_coeffsA};

// NEW DEFINES WITH iir function

#define SNR_THRESHOLD_F32 140.0f
#define BLOCK_SIZE 32
#define NUM_TAPS 29


/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24) *  * ------------------------------------------------------------------- */
float32_t *Hanning(uint32_t, uint8_t);
float32_t *avg_weighting();
float32_t *rolling_avg();
float32_t hann_window[FFT_LEN];
float32_t hann_buff[FFT_LEN];
float32_t *hann_ptr;


void PCM_to_Float(uint16_t *, float32_t *, uint16_t);
void StartFIRTask();
float32_t StartRFFTTask();
void enablefpu();
void clean_input_buffers();
void clean_output_buffers();
int timer_setup();
void fft_ws2812_Init();
void radians_to_hue(int32_t * ,uint16_t);
void Error_Handler(void);
extern void SystemClock_Config();
void calc_mag_output(float32_t *, float32_t *, uint16_t);



/* USER CODE END Private defines */


void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
