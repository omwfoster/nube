#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>

#include "stm32f4xx_hal.h"
#include "ws2812b.h"

DMA_HandleTypeDef dmaUpdate;
DMA_HandleTypeDef dmaCC1;
DMA_HandleTypeDef dmaCC2;

// Define source arrays for my DMAs
uint32_t WS2812_IO_High[] = { WS2812B_PINS };
uint32_t WS2812_IO_Low[] = { WS2812B_PINS << 16 };

WS2812_Struct * ptr_ws2812b_struct;
WS2812_Struct global_WS2812_Struct;
WS2812_BufferItem * ptr_active_output;
WS2812_BufferItem * ptr_active_input;

BB_Struct global_BB_Struct;
uint16_t ws2812bDmaBitBuffer[(24 * FFT_LEN) / 2]; // DMA output array buffer.
uint8_t frame_Buffer1[3 * FFT_LEN / 2]; // WS2812b working buffer 1
uint8_t frame_Buffer2[3 * FFT_LEN / 2]; // ws2812b working buffer 2

#define BUFFER_SIZE		(sizeof(ws2812bDmaBitBuffer)/sizeof(uint16_t))

// Gamma correction table
const uint8_t gammaTable[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
		5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12,
		12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20,
		21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32,
		32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66,
		67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
		90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
		115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138,
		140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167,
		169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
		200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233,
		236, 239, 241, 244, 247, 249, 252, 255 };

static void GPIO_init(void) {
	// WS2812B outputs
	WS2812B_GPIO_CLK_ENABLE()
	;
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = WS2812B_PINS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WS2812B_PORT, &GPIO_InitStruct);

	// Enable output pins for debuging to see DMA Full and Half transfer interrupts
#if defined(LED_BLUE_PORT) && defined(LED_ORANGE_PORT)
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = LED_BLUE_PIN;
	HAL_GPIO_Init(LED_BLUE_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LED_ORANGE_PIN;
	HAL_GPIO_Init(LED_ORANGE_PORT, &GPIO_InitStruct);
#endif
}

TIM_HandleTypeDef TIM1_handle;
TIM_OC_InitTypeDef tim2OC1;
TIM_OC_InitTypeDef tim2OC2;

uint32_t tim_period;
uint32_t timer_reset_pulse_period;

static void TIM1_init(void) {
	// TIM2 Periph clock enable
	__HAL_RCC_TIM1_CLK_ENABLE()
	;

	// This computation of pulse length should work ok,
	// at some slower core speeds it needs some tuning.
	tim_period = SystemCoreClock / 800000; // 0,125us period (10 times lower the 1,25us period to have fixed math below)
	timer_reset_pulse_period = 600; // 60us just to be sure

	uint32_t cc1 = (10 * tim_period) / 36;
	uint32_t cc2 = (10 * tim_period) / 15;

	TIM1_handle.Instance = TIM1;
	TIM1_handle.Init.Period = tim_period;
	TIM1_handle.Init.RepetitionCounter = 0;
	TIM1_handle.Init.Prescaler = 0;
	TIM1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM1_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TIM1_handle);

	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	tim2OC1.OCMode = TIM_OCMODE_PWM1;
	tim2OC1.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim2OC1.Pulse = cc1;
	tim2OC1.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	tim2OC1.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&TIM1_handle, &tim2OC1, TIM_CHANNEL_1);

	tim2OC2.OCMode = TIM_OCMODE_PWM1;
	tim2OC2.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim2OC2.Pulse = cc2;
	tim2OC2.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	tim2OC2.OCFastMode = TIM_OCFAST_DISABLE;
	tim2OC2.OCIdleState = TIM_OCIDLESTATE_RESET;
	tim2OC2.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&TIM1_handle, &tim2OC2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&TIM1_handle);
	HAL_TIM_PWM_Start(&TIM1_handle, TIM_CHANNEL_1);

	__HAL_TIM_DISABLE(&TIM1_handle);

}

DMA_HandleTypeDef dmaUpdate;
DMA_HandleTypeDef dmaCC1;
DMA_HandleTypeDef dmaCC2;


// timer for output rate
// each full cycle will execute DMA output of the Bitband buffer.




static void DMA2_init(void) {

	// TIM2 Update event
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	dmaUpdate.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dmaUpdate.Init.PeriphInc = DMA_PINC_DISABLE;
	dmaUpdate.Init.MemInc = DMA_MINC_DISABLE;
	dmaUpdate.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dmaUpdate.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	dmaUpdate.Init.Mode = DMA_CIRCULAR;
	dmaUpdate.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	dmaUpdate.Init.Channel = DMA_CHANNEL_6;
	dmaUpdate.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dmaUpdate.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	dmaUpdate.Init.MemBurst = DMA_MBURST_SINGLE;
	dmaUpdate.Init.PeriphBurst = DMA_PBURST_SINGLE;
	dmaUpdate.Instance = DMA2_Stream5;

	HAL_DMA_DeInit(&dmaUpdate);
	HAL_DMA_Init(&dmaUpdate);
	HAL_DMA_Start(&dmaUpdate, (uint32_t) WS2812_IO_High,
			(uint32_t) (&WS2812B_PORT->BSRR), BUFFER_SIZE);

	// TIM2 CC1 event
	dmaCC1.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dmaCC1.Init.PeriphInc = DMA_PINC_DISABLE;
	dmaCC1.Init.MemInc = DMA_MINC_ENABLE;
	dmaCC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaCC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	dmaCC1.Init.Mode = DMA_CIRCULAR;
	dmaCC1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	dmaCC1.Init.Channel = DMA_CHANNEL_6;
	dmaCC1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dmaCC1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	dmaCC1.Init.MemBurst = DMA_MBURST_SINGLE;
	dmaCC1.Init.PeriphBurst = DMA_PBURST_SINGLE;
	dmaCC1.Instance = DMA2_Stream1;

	HAL_DMA_DeInit(&dmaCC1);
	HAL_DMA_Init(&dmaCC1);
	HAL_DMA_Start(&dmaCC1, (uint32_t) ws2812bDmaBitBuffer,
			(uint32_t) (&WS2812B_PORT->BSRR) + 2, BUFFER_SIZE); //BRR

	// TIM2 CC2 event
	dmaCC2.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dmaCC2.Init.PeriphInc = DMA_PINC_DISABLE;
	dmaCC2.Init.MemInc = DMA_MINC_DISABLE;
	dmaCC2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dmaCC2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	dmaCC2.Init.Mode = DMA_CIRCULAR;
	dmaCC2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	dmaCC2.Init.Channel = DMA_CHANNEL_6;

	dmaCC2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dmaCC2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	dmaCC2.Init.MemBurst = DMA_MBURST_SINGLE;
	dmaCC2.Init.PeriphBurst = DMA_PBURST_SINGLE;

	dmaCC2.Instance = DMA2_Stream2;

	dmaCC2.XferCpltCallback = DMA_TransferCompleteHandler;
	dmaCC2.XferHalfCpltCallback = DMA_TransferHalfHandler;
	dmaCC2.XferErrorCallback = DMA_TransferError;

	HAL_DMA_DeInit(&dmaCC2);
	HAL_DMA_Init(&dmaCC2);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	HAL_DMA_Start_IT(&dmaCC2, (uint32_t) WS2812_IO_Low,
			(uint32_t) &WS2812B_PORT->BSRR, BUFFER_SIZE);
}

void ws2812b_init() {
	GPIO_init();

	ws2812_reset();
	DMA2_init();
	TIM1_init();
	global_WS2812_Struct.item[0].channel = 0;
	global_WS2812_Struct.item[1].channel = 0;
	global_WS2812_Struct.item[0].WS2812_buf_state = WS_NOT_IN_USE;
	global_WS2812_Struct.item[1].WS2812_buf_state = WS_NOT_IN_USE;
	global_WS2812_Struct.item[0].frameBufferCounter = 0;
	global_WS2812_Struct.item[1].frameBufferCounter = 0;
	global_WS2812_Struct.item[0].rgb_Buffer_ptr = &frame_Buffer1[0];
	global_WS2812_Struct.item[1].rgb_Buffer_ptr = &frame_Buffer2[0];
	global_WS2812_Struct.item[0].frameBufferSize = (3 * (FFT_LEN) / 2);
	global_WS2812_Struct.item[1].frameBufferSize = (3 * (FFT_LEN) / 2);
	memset(&frame_Buffer1[0],0,sizeof(frame_Buffer1));
	memset(&frame_Buffer1[0],0,sizeof(frame_Buffer2));

	//active_ws2812b_item = &ws2812b.item[0];
	// Need to start the first transfer
	global_WS2812_Struct.transferComplete = 1;

	global_BB_Struct.bb_output_state = BB_NOT_IN_USE;
	global_BB_Struct.ws2812bDmaBitBuffer = &ws2812bDmaBitBuffer[0];
//	memset(&ws2812bDmaBitBuffer[0],0,sizeof(ws2812bDmaBitBuffer) * 2);

}

void WS2812_sendbuf_helper() {

	global_BB_Struct.bb_output_state = BB_OUTPUT_BUFFER;

	// clear all DMA flags
	__HAL_DMA_CLEAR_FLAG(&dmaUpdate,
			DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5);
	__HAL_DMA_CLEAR_FLAG(&dmaCC1,
			DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5);
	__HAL_DMA_CLEAR_FLAG(&dmaCC2,
			DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6);

	// configure the number of bytes to be transferred by the DMA controller
	dmaUpdate.Instance->NDTR = BUFFER_SIZE;
	dmaCC1.Instance->NDTR = BUFFER_SIZE;
	dmaCC2.Instance->NDTR = BUFFER_SIZE;
	// clear all TIM2 flags
	__HAL_TIM_CLEAR_FLAG(&TIM1_handle,
			TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);

	// enable DMA channels
	__HAL_DMA_ENABLE(&dmaUpdate);
	__HAL_DMA_ENABLE(&dmaCC1);
	__HAL_DMA_ENABLE(&dmaCC2);

	// IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
	__HAL_TIM_ENABLE_DMA(&TIM1_handle, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&TIM1_handle, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&TIM1_handle, TIM_DMA_CC2);

	TIM1->CNT = tim_period - 1;


	__HAL_TIM_ENABLE(&TIM1_handle);
}

void DMA_TransferError(DMA_HandleTypeDef *DmaHandle) {

}

void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle) {

}

void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle) {

#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN;
#endif

	// Stop timer
	TIM1->CR1 &= ~TIM_CR1_CEN;

	// Disable DMA
	__HAL_DMA_DISABLE(&dmaUpdate);
	__HAL_DMA_DISABLE(&dmaCC1);
	__HAL_DMA_DISABLE(&dmaCC2);

	// Disable the DMA requests
	__HAL_TIM_DISABLE_DMA(&TIM1_handle, TIM_DMA_UPDATE);
	__HAL_TIM_DISABLE_DMA(&TIM1_handle, TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA(&TIM1_handle, TIM_DMA_CC2);

	// Set 50us period for Treset pulse
	//TIM2->PSC = 1000; // For this long period we need prescaler 1000
	TIM1->ARR = timer_reset_pulse_period;
	// Reset the timer
	TIM1->CNT = 0;

	// Generate an update event to reload the prescaler value immediately
	TIM1->EGR = TIM_EGR_UG;
	__HAL_TIM_CLEAR_FLAG(&TIM1_handle, TIM_FLAG_UPDATE);

	// Enable TIM2 Update interrupt for 50us Treset signal
	__HAL_TIM_ENABLE_IT(&TIM1_handle, TIM_IT_UPDATE);
	// Enable timer
	TIM1->CR1 |= TIM_CR1_CEN;

	// Manually set outputs to low to generate 50us reset impulse
	WS2812B_PORT->BSRR = WS2812_IO_Low[0];
	//READ_ws2812b_item->buf_state = NOT_IN_USE;

	global_WS2812_Struct.transferComplete = 1;
	global_BB_Struct.bb_output_state = BB_TRANSFER_COMPLETE;
	ws2812_reset();
	NVIC_EnableIRQ(TIM4_IRQn);
	BSP_AUDIO_IN_Resume();

#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN << 16;
#endif

}

void DMA2_Stream2_IRQHandler(void) {

	HAL_DMA_IRQHandler(&dmaCC2);

}

void TIM1_UP_TIM10_IRQHandler(void) {
#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN;
#endif

	HAL_TIM_IRQHandler(&TIM1_handle);

}

// TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (global_BB_Struct.bb_output_state == BB_TRANSFER_COMPLETE) {
		global_WS2812_Struct.timerPeriodCounter = 0;
		TIM1->CR1 = 0; // disable timer

		// disable the TIM2 Update IRQ
		__HAL_TIM_DISABLE_IT(&TIM1_handle, TIM_IT_UPDATE);

		// Set back 1,25us period
		TIM1->ARR = tim_period;

		// Generate an update event to reload the Prescaler value immediatly
		TIM1->EGR = TIM_EGR_UG;
		__HAL_TIM_CLEAR_FLAG(&TIM1_handle, TIM_FLAG_UPDATE);

		// set transfer_complete flag
		global_WS2812_Struct.transferComplete = 1;
		global_BB_Struct.bb_output_state = BB_NOT_IN_USE;
	}
}

// take the rgb array result, test for null, and then fill the bitchin' bit-band badness.

uint8_t BB_generator(WS2812_BufferItem volatile * WS_Buf) {



	if ((WS_Buf != NULL)
			&& ((global_BB_Struct.bb_output_state) == BB_NOT_IN_USE)) {

		global_BB_Struct.bb_output_state = BB_WRITE_LOCKED;
		WS_Buf->WS2812_buf_state = WS_READ_LOCKED;
		static uint16_t row = 0;
		static uint32_t counter = 0;
		static uint8_t red;
		static uint8_t green;
		static uint8_t blue;
		static uint8_t * rgb_ptr = 0;
		rgb_ptr = WS_Buf->rgb_Buffer_ptr;
		uint32_t *bitBand = BITBAND_SRAM(global_BB_Struct.ws2812bDmaBitBuffer,
				row); // map pointer to 0x22000000 memory space
		uint32_t volatile * bb_ptr;
		bb_ptr = bitBand;

		for (counter = 1; counter < FFT_LEN / 2; ++counter) {

			red = *rgb_ptr;
			green = *rgb_ptr++;
			blue = *rgb_ptr++;
			// Apply gamma
			//	red = gammaTable[red];
			//	green = gammaTable[green];
			//	blue = gammaTable[blue];
			uint32_t invRed = ~red;
			uint32_t invGreen = ~green;
			uint32_t invBlue = ~blue;

			// Bitband optimisations with pure increments, 5us interrupts

			*bb_ptr = (invGreen >> 7);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 6);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 5);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 4);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 3);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 2);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 1);
			bb_ptr += 16;

			*bb_ptr = (invGreen >> 0);
			bb_ptr += 16;

			// RED
			*bb_ptr = (invRed >> 7);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 6);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 5);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 4);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 3);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 2);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 1);
			bb_ptr += 16;

			*bb_ptr = (invRed >> 0);
			bb_ptr += 16;

			// BLUE
			*bb_ptr = (invBlue >> 7);
			bb_ptr += 16;

			*bb_ptr = (invBlue >> 6);
			bb_ptr += 16;

			*bb_ptr = (invBlue >> 5);
			bb_ptr += 16;

			*bb_ptr = (invBlue >> 4);
			bb_ptr += 16;

			*bb_ptr = (invBlue >> 3);
			bitBand += 16;

			*bitBand = (invBlue >> 2);
			bitBand += 16;

			*bitBand = (invBlue >> 1);
			bitBand += 16;

			*bitBand = (invBlue >> 0);
			bitBand += 16;



		}

		row = 0;
		counter = 0;
		red = 0;
		green = 0;
		blue = 0;
		rgb_ptr = NULL;
		global_BB_Struct.bb_output_state = BB_BUFFER_READY;
		WS_Buf->WS2812_buf_state = WS_NOT_IN_USE;
		return 1; //default
	}
	return 0 ;
}

// kick off DMA output
// if BitBand buffer has been filled by the main process.
// bb_output_state will be         BB_BUFFER_READY

uint8_t ws2812b_handle() {


	if (global_BB_Struct.bb_output_state == BB_BUFFER_READY) {
		NVIC_DisableIRQ(TIM4_IRQn);
		BSP_AUDIO_IN_Pause();
		WS2812_sendbuf_helper();
		return 1;
	}
	return 0;



}

void ws2812_reset() {

	// Set 50us period for Treset pulse
	//TIM2->PSC = 1000; // For this long period we need prescaler 1000
	TIM1->ARR = timer_reset_pulse_period;
	// Reset the timer
	TIM1->CNT = 0;

	// Generate an update event to reload the prescaler value immediately
	TIM1->EGR = TIM_EGR_UG;
	__HAL_TIM_CLEAR_FLAG(&TIM1_handle, TIM_FLAG_UPDATE);

	// Enable TIM2 Update interrupt for 50us Treset signal
	__HAL_TIM_ENABLE_IT(&TIM1_handle, TIM_IT_UPDATE);
	// Enable timer
	TIM1->CR1 |= TIM_CR1_CEN;

	// Manually set outputs to low to generate 50us reset impulse
	WS2812B_PORT->BSRR = WS2812_IO_Low[0];

}

// retrieve an RGB buffer according to status enumeration
// buf_state {READ_LOCKED,WRITE_LOCKED,BUFFER_FULL,NOT_IN_USE}
// if no suitable buffer is available then return NULL pointer.

WS2812_BufferItem * ws2812b_getBufferItem(ws_buf_state status) {



	if (global_WS2812_Struct.item[0].WS2812_buf_state == status) {

		return &global_WS2812_Struct.item[0];
	}
	if (global_WS2812_Struct.item[1].WS2812_buf_state == status) {

		return &global_WS2812_Struct.item[1];
	}
	return NULL;




}
#ifdef __cplusplus
}
#endif

