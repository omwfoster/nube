#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;

#include "visEffect.h"
#include "stdbool.h"
#include "omwof/omwof_test.h"
#include "omwof/omwof_weight.h"
#include "omwof/omwof_window.h"
#include "omwof/omwof_menu.h"
#include "omwof/omwof_button.h"
#include "omwof/omwof_db.h"

#define OUTPUT_TEST
SPI_HandleTypeDef hspi1;

I2S_HandleTypeDef hAudioInI2s;
I2C_HandleTypeDef SSD1306_I2C_PORT;

static volatile uint32_t ITCounter = 0; //  buffer position for use in isr
static volatile uint16_t buff_pos = 0;  // pointer to buffer position

float32_t F_Sum = 0.0f;  // cmsis support function variables
float32_t max_Value = 0.0;
uint32_t max_Index = 0;
float32_t st_dev;
float32_t st_max = 63566;
float32_t st_min = 0;

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

static const uint16_t SAMPLE_RUNS = 32; //(INTERNAL_BUFF_SIZE / PCM_OUT_SIZE);

float32_t fft_input_array[FFT_LEN]; //
float32_t fft_temp_array[FFT_LEN]; //
float32_t fft_output_bins[FFT_LEN];
float32_t mag_output_bins[FFT_LEN / 2]; //
float32_t db_output_bins[FFT_LEN / 2]; //

chunk_TypeDef sine_test;

// hanning windowing functions for sample blocks.
float32_t array_window[FFT_LEN];
float32_t hann_buff[FFT_LEN];

// sample data
uint16_t internal_buffer[INTERNAL_BUFF_SIZE]; // read raw pdm input    128 * DEFAULT_AUDIO_IN_FREQ/16000 *DEFAULT_AUDIO_IN_CHANNEL_NBR
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

TIM_HandleTypeDef TIM_Handle;

uint8_t TIM4_config(void)

{
	__TIM4_CLK_ENABLE()
	;
	TIM_Handle.Init.Prescaler = 100;
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

#define UDG	0

extern menu_typedef * toplevel_menu[3];


void add_ui() {

	static callback_typedef power_list[] = {
			{.callback_ptr = (typedef_func_union ) &power_spectra , .callback_name = "power1",},
			{.callback_ptr = (typedef_func_union ) &power_spectra1  , .callback_name = "power2",},
			{.callback_ptr = (typedef_func_union ) &power_spectra2 , .callback_name = "power3",},
			{.callback_ptr = (typedef_func_union ) &power_spectra3  , .callback_name = "power4",},
			{.callback_ptr = (typedef_func_union ) &power_spectra4  , .callback_name = "power5",},

	};

	add_new_menu(&power_list[0],5,"Powers",0,POWER_FOLDER);

	static callback_typedef weight_list[] = {
			{.callback_ptr = (typedef_func_union ) &rms_weighting , .callback_name = "bad_rms_1",},
			{.callback_ptr = (typedef_func_union ) &rms_weighting_2 , .callback_name = "good_rms_2",},
			{.callback_ptr = (typedef_func_union ) &sd_weighting , .callback_name = "sd_1",},
			{.callback_ptr = (typedef_func_union ) &sd_weighting_2 , .callback_name = "sd_2",},
		};

		add_new_menu(&weight_list[0],4,"Weights",1,WEIGHT_FOLDER);

	static callback_typedef window_list[] = {
			{.callback_ptr = (typedef_func_union)&Hanning , .callback_name = "Hanning",},
			{.callback_ptr = (typedef_func_union)&Hamming , .callback_name = "Hamming",},
			{.callback_ptr = (typedef_func_union)&Blackman , .callback_name = "Blackman",},
			{.callback_ptr = (typedef_func_union)&Kaiser , .callback_name = "Kaiser",},
			{.callback_ptr = (typedef_func_union)&Chebeyshev , .callback_name = "Chebayshev",},

	};
	add_new_menu(&window_list[0],5,"Windows",2,WINDOW_FOLDER);
}


static void MX_I2C2_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
}



static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
}

/* You can connect I2C1 to two different


 * pairs of pins:


 * 1. SCL on PB6 and SDA on PB7


 * 2. SCL on PB8 and SDA on PB9


 GPIO_Pin_6, GPIO_Pin_7 for PB6 and PB7


 GPIO_Pin_8, GPIO_Pin_9 for PB8 and PB9 */

__IO uint8_t UserPressButton = 0;
uint8_t weight_profile_index = 0;

void init() {

	enablefpu();
	HAL_Init();
	MX_GPIO_Init();
	MX_I2C2_Init();
	BSP_Audio_init();
	ssd1306_Init();
	cleanbuffers();
	add_ui();
	//ssd1306_TestAll();
	fft_ws2812_Init();
	init_button();
}

void main(void) {

	init();

	set_window(); // create an array containing window coefficients for the input array

	TIM4_config(); // timer for LED refresh
	BSP_AUDIO_IN_SetVolume(64);
	UserPressButton = 0;
	float32_t weight = 0.0f;

	while (1) {

		if ((AUDIODataReady == 1 && FFT_Ready == 0)) {
			StartRFFTTask();   // test if duration variable is overwritten
		}

		if ((FFT_Ready == 1) && (LED_Ready == 0)) {
			weight = toplevel_menu[1]
								   ->active_callback
								   ->callback_ptr.func_weight(&mag_output_bins[0], (FFT_LEN / 2), &st_dev);

			generate_RGB(&fft_output_bins[0], &mag_output_bins[0],
					&db_output_bins[0], (FFT_LEN / 2), weight);
			LED_Ready = generate_BB();
		}
		if (get_BB_status(BB_TRANSFER_COMPLETE) == 1) {
			LED_Ready = 0;
			FFT_Ready = 0;
			AUDIODataReady = 0;
		}
	}

}

void drop_volume() {

	static uint8_t volume = 128;
	if (volume > 8) {
		BSP_AUDIO_IN_SetVolume(volume);
		volume /= 2;
	} else {
		volume = 128;
		BSP_AUDIO_IN_SetVolume(volume);
		volume = 128;

	}
}

void BSP_Audio_init() {

	BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ,
	DEFAULT_AUDIO_IN_BIT_RESOLUTION,
	DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t *) &internal_buffer[0],
	INTERNAL_BUFF_SIZE); // start reading pdm data into buffer

}

void BSP_Led_init() {
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

}



void set_window() {
	toplevel_menu[2]->active_callback->callback_ptr.func_window(&array_window[0], FFT_LEN);

}

void fft_ws2812_Init() {
	set_window(&array_window[0],FFT_LEN);
	arm_rfft_fast_init_f32(&rfft_s, FFT_LEN);
	AUDIODataReady = 0;
	BSP_Audio_init();
	visInit();
}

float32_t max_db;
float32_t min_db;

uint8_t StartRFFTTask() {

	arm_rfft_fast_f32(&rfft_s, &fft_input_array[0], &fft_output_bins[0], 0);
	arm_cmplx_mag_f32(&fft_output_bins[0], &mag_output_bins[0], (FFT_LEN / 2));
	toplevel_menu[0]->active_callback->callback_ptr.func_power(
			&mag_output_bins[0], &db_output_bins[0], (FFT_LEN / 2));
	arm_fill_f32(0.0f, &fft_input_array[0], FFT_LEN);
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
				(uint16_t *) &internal_buffer[INTERNAL_BUFF_SIZE / 2],
				(uint16_t *) &PCM_Buf[0]);

		PCM_to_Float((uint16_t *) &PCM_Buf[0], (float32_t *) &float_array[0],
		PCM_OUT_SIZE);
		arm_mult_f32(&float_array[0], &array_window[buff_pos],
				&fft_input_array[buff_pos], PCM_OUT_SIZE);

		if (ITCounter < (SAMPLE_RUNS - 1)) {
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
		BSP_AUDIO_IN_PDMToPCM((uint16_t *) &internal_buffer[0],
				(uint16_t *) &PCM_Buf[0]);
		PCM_to_Float((uint16_t *) &PCM_Buf[0], (float32_t *) &float_array[0],
		PCM_OUT_SIZE);
		arm_mult_f32(&float_array[0], &array_window[buff_pos],
				&fft_input_array[buff_pos], PCM_OUT_SIZE);

		if (ITCounter < (SAMPLE_RUNS - 1)) {
			ITCounter++;

		} else {
			AUDIODataReady = 1;
			ITCounter = 0;

		}
	}
}

void I2S2_IRQHandler(void) {
	HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}



void cleanbuffers() {


	arm_fill_f32(0.0f, fft_output_bins, FFT_LEN);
	arm_fill_f32(0.0f, mag_output_bins, FFT_LEN / 2);

}

void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

