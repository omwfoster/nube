#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
//I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;

#include "visEffect.h"
#include "stdbool.h"
#include "omwof/omwof_test.h"

#define OUTPUT_TEST
SPI_HandleTypeDef hspi1;

I2S_HandleTypeDef hAudioInI2s;

static volatile uint32_t ITCounter = 0; //  buffer position for use in isr
volatile uint16_t buff_pos = 0;  // pointer to buffer position

float32_t F_Sum = 0.0f;  // cmsis support function variables
float32_t max_Value = 0.0;
uint32_t max_Index = 0;

/* Save MEMS ID */
uint8_t MemsID = 0;
/* Buffer Status variables */

volatile uint8_t AUDIODataReady = 0, FFT_Ready = 0, LED_Ready = 0, PDM_Running =
		0;

arm_rfft_fast_instance_f32 rfft_s;

/* Width of the peak */
static float32_t peq1_width = 5.0f;
static float32_t peq1_coeffsA[5] = { 1.0, -1.0, 0, 0.95, 0 };

static float32_t peq1_state[2];
static bool peq1_abFlag = false;
/* Two filter instances so we can use one while calculating the coefficients of the other */
static const arm_biquad_casd_df1_inst_f32 peq1_instanceA = { 1, peq1_state,
		peq1_coeffsA };

static volatile uint16_t SAMPLE_RUNS = 7;    //(INTERNAL_BUFF_SIZE / PCM_OUT_SIZE);

float32_t BQ_Input[FFT_LEN]; //

float32_t FFT_Input[FFT_LEN]; //
float32_t FFT_Bins[FFT_LEN];
float32_t FFT_MagBuf[FFT_LEN / 2]; //
float32_t FFT_MagBuf_IIR[FFT_LEN / 2]; //

chunk_TypeDef sine_test;

// hanning windowing functions for sample blocks.
float32_t hann_window[FFT_LEN];
float32_t hann_buff[FFT_LEN];

// sample data
uint16_t InternalBuffer[INTERNAL_BUFF_SIZE]; // read raw pdm input    128 * DEFAULT_AUDIO_IN_FREQ/16000 *DEFAULT_AUDIO_IN_CHANNEL_NBR
uint16_t PCM_Buf[PCM_OUT_SIZE]; //PCM stereo samples are saved in RecBuf  DEFAULT_AUDIO_IN_FREQ/1000
float32_t float_array[PCM_OUT_SIZE];



void enablefpu() {
	__asm volatile(
			"  ldr.w r0, =0xE000ED88    \n" /* The FPU enable bits are in the CPACR. */
			"  ldr r1, [r0]             \n" /* read CAPCR */
			"  orr r1, r1, #( 0xf <<20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
			"  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
			"  dsb                       \n" /* wait for store to complete */
			"  isb" /* reset pipeline now the FPU is enabled */);
}



void cleanbuffers() {

#ifndef OUTPUT_TEST

	arm_fill_f32(0.0f, FFT_Input, FFT_LEN);

#endif
	arm_fill_f32(0.0f, BQ_Input, FFT_LEN);
	arm_fill_f32(0.0f, FFT_Bins, FFT_LEN);
	arm_fill_f32(0.0f, FFT_MagBuf, FFT_LEN / 2);

}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

TIM_HandleTypeDef TIM_Handle;

uint8_t TIM4_config(void)

{

	__TIM4_CLK_ENABLE()
	;
	TIM_Handle.Init.Prescaler = 40;
	TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle.Init.Period = 16000;
	TIM_Handle.Instance = TIM4;   //Same timer whose clocks we enabled
	HAL_TIM_Base_Init(&TIM_Handle);     // Init timer
	HAL_TIM_Base_Start_IT(&TIM_Handle); // start timer interrupts
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	return 1;

}
void TIM4_IRQHandler(void)

{
	if (__HAL_TIM_GET_FLAG(&TIM_Handle, TIM_FLAG_UPDATE) != RESET) //In case other interrupts are also running
			{
		if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_FLAG(&TIM_Handle, TIM_FLAG_UPDATE);

			ws2812b_handle();
		}
	}
}

volatile uint32_t i = 4; // start at first useful value. wavelength = 4samples -- 00 -- up -- 00 -- down
void test_loop2() {

	if (i <= FFT_LEN) {
		sine_sample(&FFT_Input[0], FFT_LEN, i); //  calculate sine values for a wave run
		i *= 2; 								//  multiply by 2 for next run
	} else {
		i = 4;									//  restart sequence if the sequence were to overflow the
												//	overall sample length
	}

	AUDIODataReady = 1;
}

uint8_t volume = 128;

void main(void) {
	cleanbuffers();
	fft_ws2812_Init();
	TIM4_config(); // timer for LED refresh
	BSP_AUDIO_IN_SetVolume(128);


	while (1) {

	//	if (AUDIODataReady == 0) {
	//		test_loop2();

	//	}

		if ((AUDIODataReady == 1 && FFT_Ready == 0)) {
			StartRFFTTask();
		}

		if ((FFT_Ready == 1) && (LED_Ready == 0)) {
			generate_RGB(&FFT_Bins[0], &FFT_MagBuf[0], (FFT_LEN / 2));
			LED_Ready = generate_BB();
		}
		if (get_BB_status() == 1) {
			LED_Ready = 0;
			FFT_Ready = 0;
			AUDIODataReady = 0;
			volume /= 2;
			BSP_AUDIO_IN_SetVolume(volume);
		}

	}

}

void BSP_Audio_init() {

	BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ,
	DEFAULT_AUDIO_IN_BIT_RESOLUTION,
	DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t *) &InternalBuffer[0],
	INTERNAL_BUFF_SIZE); // start reading pdm data into buffer

}

void BSP_Led_init() {
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);
	BSP_LED_On(LED4);
}

void fft_ws2812_Init() {


	enablefpu();
	HAL_Init();
	hann_ptr = Hanning((FFT_LEN), 1);
	arm_rfft_fast_init_f32(&rfft_s, FFT_LEN);

	AUDIODataReady = 0;
	BSP_Audio_init();
	visInit();

}

uint8_t StartRFFTTask() {

	BSP_LED_Toggle(LED5);

	arm_rfft_fast_f32(&rfft_s, &FFT_Input[0], &FFT_Bins[0], 0);
	arm_cmplx_mag_f32(&FFT_Bins[0], &FFT_MagBuf[0], (FFT_LEN / 2));
	arm_fill_f32(0.0f, FFT_Input, FFT_LEN);

	FFT_Ready = 1;
	return 1;

}

void PCM_to_Float(uint16_t *samples_PCM, float32_t *samples_float32,
		uint16_t length_array) {

	for (uint16_t i = 0; i < length_array; i++) {
		samples_float32[i] = (float32_t) samples_PCM[i];
	}

}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {

	if (AUDIODataReady == 0) {
		buff_pos = ITCounter * PCM_OUT_SIZE;
		BSP_AUDIO_IN_PDMToPCM(
				(uint16_t *) &InternalBuffer[INTERNAL_BUFF_SIZE / 2],
				(uint16_t *) &PCM_Buf[0]);

		PCM_to_Float((uint16_t *) &PCM_Buf[0], (float32_t *) &float_array[0],
		PCM_OUT_SIZE);
		arm_mult_f32(&float_array[0], &hann_window[buff_pos],
				&FFT_Input[buff_pos], PCM_OUT_SIZE);

		if (ITCounter < SAMPLE_RUNS) {
			ITCounter++;

		} else {
			AUDIODataReady = 1;
			ITCounter = 0;

		}
	}
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
	if (AUDIODataReady == 0) {
		buff_pos = ITCounter * PCM_OUT_SIZE;
		/* PDM to PCM data convert */
		BSP_AUDIO_IN_PDMToPCM((uint16_t *) &InternalBuffer[0],
				(uint16_t *) &PCM_Buf[0]);
		PCM_to_Float((uint16_t *) &PCM_Buf[0], (float32_t *) &float_array[0],
		PCM_OUT_SIZE);
		arm_mult_f32(&float_array[0], &hann_window[buff_pos],
				&FFT_Input[buff_pos], PCM_OUT_SIZE);

		if (ITCounter < (SAMPLE_RUNS)) {
			ITCounter++;

		} else {
			AUDIODataReady = 1;
			ITCounter = 0;

		}
	}
}

float32_t *Hanning(uint32_t N, uint8_t itype) {
	uint32_t half, i, idx, n;

	arm_fill_f32(0.0f, &hann_window[0], N);

	if (itype == 1) //periodic function
		n = N - 1;
	else
		n = N;

	if (n % 2 == 0) {
		half = n / 2;
		for (i = 0; i < half; i++) //CALC_HANNING   Calculates Hanning window samples.
			hann_window[i] = 0.5 * (1 - cos(2 * PI * (i + 1) / (n + 1)));

		idx = half - 1;
		for (i = half; i < n; i++) {
			hann_window[i] = hann_window[idx];
			idx--;
		}
	} else {
		half = (n + 1) / 2;
		for (i = 0; i < half; i++) //CALC_HANNING   Calculates Hanning window samples.
			hann_window[i] = 0.5 * (1 - cos(2 * PI * (i + 1) / (n + 1)));

		idx = half - 2;
		for (i = half; i < n; i++) {
			hann_window[i] = hann_window[idx];
			idx--;
		}
	}

	if (itype == 1) //periodic function
			{
		for (i = N - 1; i >= 1; i--)
			hann_window[i] = hann_window[i - 1];
		hann_window[0] = 0.0;
	}
	return (&hann_window[0]);
}

void BSP_AUDIO_IN_Error_Callback(void) {
	Error_Handler();
}

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void) {
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) {
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) {
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s).                                              */
/******************************************************************************/

/**
 * @brief  This function handles DMA Stream interrupt request.
 * @param  None
 * @retval None
 */
void I2S2_IRQHandler(void) {
	HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
