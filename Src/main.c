#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "visEffect.h"

//I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

I2S_HandleTypeDef hAudioInI2s;



filter_typedef FUNCTION = RFFT;

volatile uint32_t ITCounter = 0; //  buffer position
volatile uint16_t buff_pos = 0;

float32_t F_Sum = 0.0f;
float32_t max_Value = 0.0;
uint32_t max_Index = 0;

/* Save MEMS ID */
uint8_t MemsID = 0;
/* Buffer Status variables */

volatile uint8_t AUDIODataReady = 0, FFT_Ready = 0, LED_Ready = 0, PDM_Running = 0;

float32_t FFT_Input[FFT_LEN]; //
float32_t FFT_Bins[FFT_LEN];
float32_t FFT_MagBuf[FFT_LEN / 2]; //
float32_t FFT_MagBuf_IIR[FFT_LEN / 2]; //
float32_t FIR_Buf[FFT_LEN];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void) {

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	fft_ws2812_Init();
	TIM4_config();


	while (1) {



		if (AUDIODataReady == 1) {
			StartRFFTTask();
		}
		if (FFT_Ready == 1) {
			generate_RGB(&FFT_Bins[0], &FFT_MagBuf[0], (FFT_LEN / 2));
			FFT_Ready = 0;
		}

		LED_Ready = generate_BB();


	}
	return 1;
	/* USER CODE END 3 */
}



void fft_ws2812_Init() {



	sample_runs = 15;
	enablefpu();
	HAL_Init();

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);
	BSP_LED_On(LED4);
	hann_ptr = Hanning((FFT_LEN), 1);
	arm_rfft_fast_init_f32(&rfft_s, FFT_LEN);

#ifndef TEST2

	BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION,
	DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t *) &InternalBuffer[0], INTERNAL_BUFF_SIZE); // start reading pdm data into buffer

#endif

	visInit();

}

uint8_t StartRFFTTask() {

	BSP_LED_Toggle(LED5);
	arm_rfft_fast_f32(&rfft_s, &FFT_Input[0], &FFT_Bins[0], 0);
	arm_cmplx_mag_f32(&FFT_Bins[0], &FFT_MagBuf[0], (FFT_LEN / 2));
	//calc_mag_output(&FFT_MagBuf_IIR[0], &FFT_MagBuf[0], FFT_LEN / 2);
	AUDIODataReady = 0;
	FFT_Ready = 1;
	return 1;

}

void PCM_to_Float(uint16_t *samples_PCM, float32_t *samples_float32,
		uint16_t length_array) {

	for (uint16_t i = 0; i < length_array; i++) {
		samples_float32[i] = (float32_t) samples_PCM[i] * (1.0f / 65535.0f);
	}

}

void PCM_Preprocess(float32_t *input_ptr, float32_t *window,
		float32_t *output_ptr, uint8_t length_block) {

	arm_mult_f32(&input_ptr[0], &window[0], &output_ptr[0],
			(uint8_t) length_block);

}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {

	if (AUDIODataReady == 0) {
		buff_pos = ITCounter * PCM_OUT_SIZE;
		BSP_AUDIO_IN_PDMToPCM(
				(uint16_t *) &InternalBuffer[INTERNAL_BUFF_SIZE / 2],
				(uint16_t *) &PCM_Buf[0]);

		PCM_to_Float((uint16_t *) &PCM_Buf[0], (float32_t *) &float_array[0],
		PCM_OUT_SIZE);
		PCM_Preprocess((float32_t *) &float_array[0],
				(float32_t *) &hann_window[ITCounter * PCM_OUT_SIZE],
				(float32_t *) &FFT_Input[ITCounter * PCM_OUT_SIZE],
				PCM_OUT_SIZE);

		if (ITCounter == sample_runs) {
			AUDIODataReady = 1;
			ITCounter = 0;
		} else {
			ITCounter++;
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
		PCM_Preprocess((float32_t *) &float_array[0],
				(float32_t *) &hann_window[ITCounter * PCM_OUT_SIZE],
				(float32_t *) &FFT_Input[ITCounter * PCM_OUT_SIZE],
				PCM_OUT_SIZE);

		if (ITCounter == sample_runs) {
			AUDIODataReady = 1;
			ITCounter = 0;
		} else {
			ITCounter++;
		}
	}
}

void calc_mag_output(float32_t * mag_old, float32_t * mag_new, uint16_t len) {

	float32_t * m_o;
	float32_t * m_n;
	m_n = mag_new;
	m_o = mag_old;
	for (uint16_t i = 1; i < len; ++i) {
		*m_o = ((*(m_n) * 0.02F) + (*(m_o) * 0.98F));
		m_n++;
		m_o++;
	}
}

// Enable the FPU (Cortex-M4 - STM32F4xx and higher)
// http://infocenter.arm.com/help/topic/com.arm.doc.dui0553a/BEHBJHIG.html
void enablefpu() {
	__asm volatile(
			"  ldr.w r0, =0xE000ED88    \n" /* The FPU enable bits are in the CPACR. */
			"  ldr r1, [r0]             \n" /* read CAPCR */
			"  orr r1, r1, #( 0xf <<20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
			"  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
			"  dsb                       \n" /* wait for store to complete */
			"  isb" /* reset pipeline now the FPU is enabled */);
}





float32_t *Hanning(uint32_t N, uint8_t itype) {
	uint32_t half, i, idx, n;

	memset(hann_window, 0, (N * 4));

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
