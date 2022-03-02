/* Includes ------------------------------------------------------------------*/
#include "uart_send.h"
#include <main.h>
#include <stm32f4xx_hal.h>
#include <usb_device.h>
#include <port.h>

#include "sensniff.h"


extern UART_HandleTypeDef huart3;


#define UART_DMA_BUFFER_SIZE 1024
static uint8_t uart_dma_bytes[UART_DMA_BUFFER_SIZE];
/// Number of bytes currently sent over dma.
static uint8_t dma_sending = 0;
static int pending_bytes = 0;
static int pending_pos = 0;

void dma_still_sending_error(void) {
	UNUSED("UART was too slow");
}


void uart_dma_send(uint8_t *bytes_to_send, int uart_len) {

	if (dma_sending == 0) {
		// Copy the bytes to our buffer.
		memcpy(&uart_dma_bytes, bytes_to_send, uart_len);
		dma_sending = uart_len;
		// Then transmit the data using non-blocking UART DMA
		HAL_UART_Transmit_DMA(&huart3, uart_dma_bytes, uart_len);

	}else {
		//Already sending
		// Append the bytes to the uart bytes
		int pos_write;
		if (pending_pos == 0) {
			pending_pos = dma_sending;
			pos_write = pending_pos;
		}else {
			pos_write = pending_pos + pending_bytes;
		}

		pending_bytes += uart_len;

		if (pending_pos + uart_len > UART_DMA_BUFFER_SIZE) {
			//Failed. Buffer is full
			return;
		}

		memcpy(&uart_dma_bytes[pos_write], bytes_to_send, uart_len);
	}
}

/**
 * UART finished sending. If the buffer has more bytes, send them as well
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	//UART finished transmitting

	UNUSED(huart);

	if (pending_bytes > 0) {
		dma_sending = pending_bytes;
		HAL_UART_Transmit_DMA(&huart3, &uart_dma_bytes[pending_pos], pending_bytes);
		pending_bytes = 0;
	}else {
		dma_sending = 0;
	}

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	//UART Error
	dma_sending = 0;
	UNUSED(huart);
}


/*!
 * Log any string over UART. If the setup is using Sensniff, a sensniff debug log will be sent.
 */
void log_uart(char *data) {
//	uint16_t data_length=strlen((const char *)data) + 1;
//	char message[data_length];
//	sprintf(message, "%s\n", data);
	if (USE_SENSNIFF_FOR_LOGS) {
		sensniff_debug_log((unsigned char *) data);
	}else {
		uint16_t data_length=strlen((const char *)data);
		uart_dma_send((uint8_t*) data, data_length);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn test_run_info()
*
* @brief  This gets run info from a test and sends it through virtual COM port.
*
* @param data - Message data, this data should be NULL string.
*
* output parameters
*
* no return value
*/
void test_run_info(unsigned char *data)
{
	if (USE_SENSNIFF_FOR_LOGS) {
		sensniff_debug_log(data);
	}

	uint16_t data_length=strlen((const char *)data);
	uart_dma_send((uint8_t*) data, data_length);
}

