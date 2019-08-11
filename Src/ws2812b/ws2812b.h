
#ifndef WS2812B_H_
#define WS2812B_H_
#define FFT_LEN 128


// GPIO enable command
#define WS2812B_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
// LED output port
#define WS2812B_PORT GPIOC
// LED output pins
#define WS2812B_PINS (GPIO_PIN_0)

// How many LEDs are in the series - only valid multiples by two
#define WS2812B_NUMBER_OF_LEDS 64


#define WS2812_BUFFER_COUNT 2

// Choose one of the bit-juggling setpixel implementation
// *******************************************************
//#define SETPIX_1	// For loop, works everywhere, slow
//#define SETPIX_2	// Bit band in a loop
//#define SETPIX_3	// Like SETPIX_1 but with unrolled loop
#define SETPIX_4	// Fastest copying using bit-banding


// DEBUG OUTPUT
// ********************

// Set during DMA Half and Full transfer IRQ to debug how long IRQ is processing
#define LED_BLUE_PORT GPIOD
#define LED_BLUE_PIN GPIO_PIN_15

// Set during full transfer DMA and TIM1 IRQ
#define LED_ORANGE_PORT GPIOD
#define LED_ORANGE_PIN GPIO_PIN_13


// Public functions
// ****************
void ws2812b_init();
uint8_t ws2812b_handle();

// Library structures
// ******************
// This value sets number of periods to generate 50uS Treset signal
#define WS2812_RESET_PERIOD 50
typedef enum buf_state {WS_READ_LOCKED,WS_WRITE_LOCKED,WS_BUFFER_FULL,WS_NOT_IN_USE,TEST} ws_buf_state;

typedef struct WS2812_BufferItem {
	uint32_t frameBufferSize;
	uint32_t frameBufferCounter;
	uint8_t channel;	// digital output pin/channel
	ws_buf_state WS2812_buf_state;
	uint8_t* rgb_Buffer_ptr;
} __attribute__((packed)) WS2812_BufferItem;



typedef struct WS2812_Struct
{
	WS2812_BufferItem item[WS2812_BUFFER_COUNT];
	uint8_t transferComplete;
	uint32_t timerPeriodCounter;
	uint32_t repeatCounter;
} __attribute__((packed)) WS2812_Struct;




typedef enum bb_buf_state {BB_HALF_TRANSFER,BB_TRANSFER_COMPLETE,BB_OUTPUT_BUFFER,BB_BUFFER_READY,BB_WRITE_LOCKED,BB_NOT_IN_USE} bb_buf_state;

typedef struct BitBand_Buffer
{
	bb_buf_state bb_output_state;
	uint16_t * ws2812bDmaBitBuffer;
} __attribute__((packed)) BB_Struct;


extern WS2812_Struct * ptr_ws2812b_struct;
extern WS2812_Struct   BB;


// Bit band stuff
#define RAM_BASE 0x20000000
#define RAM_BB_BASE 0x22000000
#define Var_ResetBit_BB(VarAddr, BitNumber) (*(volatile uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 0)
#define Var_SetBit_BB(VarAddr, BitNumber) (*(volatile uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 1)
#define Var_GetBit_BB(VarAddr, BitNumber) (*(volatile uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)))
#define BITBAND_SRAM(address, bit) ( (uint32_t *) (RAM_BB_BASE + (((uint32_t)address) - RAM_BASE) * 32 + (bit) * 4))


//void create_WS2812_Struct();
void ws2812b_set_pixel(uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue);
WS2812_BufferItem * ws2812b_getBufferItem(ws_buf_state status);
#define varSetBit(var,bit) (Var_SetBit_BB((uint32_t)&var,bit))
#define varResetBit(var,bit) (Var_ResetBit_BB((uint32_t)&var,bit))
#define varGetBit(var,bit) (Var_GetBit_BB((uint32_t)&var,bit))
uint8_t BB_generator(WS2812_BufferItem volatile  * WS_Buf);

void ws2812_reset();
void WS2812_sendbuf_helper();
void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle);
void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle);
void DMA_TransferError(DMA_HandleTypeDef *DmaHandle);


void TIM4_vishandle_wrapper();
uint8_t TIM4_config();


#ifdef __cplusplus
}
#endif


#endif /* VISEFFECT_H_ */

