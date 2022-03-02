// Sensniff support functions for
#include "uart_send.h"
#include "sensniff.h"
#include <string.h>
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <deca_types.h>


extern UART_HandleTypeDef huart3;

static uint8_t sensniff_bytes[] = {0xC1, 0x1F, 0xFE, 0x72};

/*!
 *  @fn	sensniff_rx_frame()
 *
 *  @brief	This function sends a received frame to a receiver running sensniff
 *
 *  @param rx_buffer - The frame content as a rx buffer
 *  @param frame_len - The length of the frame
 *
 *  no return value
 */
void sensniff_rx_frame(uint8_t* rx_buffer, int frame_len) {
	uint8_t bytes_to_send[8 + frame_len];
	memcpy(bytes_to_send, sensniff_bytes, 4);
	bytes_to_send[4] = 2; //Sensniff version
	bytes_to_send[5] = 0x00; // Frame CMD

	bytes_to_send[6] = (frame_len >> 8);
	bytes_to_send[7] = frame_len & 0xFF;

	//Copy the data
	memcpy(&bytes_to_send[8], rx_buffer, frame_len);
	int uart_len = 8 + frame_len;

	uart_dma_send(bytes_to_send, uart_len);


//	HAL_UART_Transmit(&huart3, bytes_to_send, 8 + frame_len, 10);
//	HAL_UART_Transmit_DMA(&huart3, bytes_to_send, 8 + frame_len);
}

void sensniff_rx_frame_timestamp(uint8_t* rx_buffer, int frame_len, uint64_t timestamp) {

	// 8 Bytes Sensniff header + Frame + 8 bytes timestamp
	int uart_len = 8 + frame_len + 8;
	uint8_t bytes_to_send[uart_len];
	memcpy(bytes_to_send, sensniff_bytes, 4);
	bytes_to_send[4] = 2; //Sensniff version
	bytes_to_send[5] = 0x20; // Frame Timestamp CMD

	bytes_to_send[6] = ((frame_len + 8) >> 8);
	bytes_to_send[7] = (frame_len + 8) & 0xFF;

	//Copy the data
	memcpy(&bytes_to_send[8], rx_buffer, frame_len);

	//Copy the timestamp
	int pos = 8 + frame_len;
	memcpy(&bytes_to_send[pos], &timestamp, sizeof(timestamp));

	uart_dma_send(bytes_to_send, uart_len);
}



/*!
 *  @fn	sensniff_debug_log()
 *
 *  @brief	This function sends a debug message to a receiver that runs sensniff
 *
 *  @param data - Message data, this should be a NULL terminated string
 *
 *  no return value
 */
void sensniff_debug_log(unsigned char *data) {
	uint16_t    data_length;
	data_length=strlen((const char *)data);

	uint16_t bytes_len = data_length + 2;

	uint8_t bytes_to_send[8 + bytes_len];
	memcpy(bytes_to_send, sensniff_bytes, 4);
	bytes_to_send[4] = 2; //Sensniff version
	bytes_to_send[5] = 0x10; // Debug Print CMD

	bytes_to_send[6] = (bytes_len >> 8);
	bytes_to_send[7] = bytes_len & 0xFF;

	bytes_to_send[8 + bytes_len-2] = (uint8_t)'\n';
	bytes_to_send[8 + bytes_len-1] = (uint8_t)'\r';

	//Copy the data
	memcpy(&bytes_to_send[8], data, data_length);
//	int uart_len = 8 + bytes_len;


//	uart_dma_send(bytes_to_send, uart_len);
}

void sensniff_reply(unsigned char *data, uint16_t data_length,  uint8_t cmd) {

	uint8_t bytes_to_send[8 + data_length];
	memcpy(bytes_to_send, sensniff_bytes, 4);
	bytes_to_send[4] = 2; //Sensniff version
	bytes_to_send[5] = cmd; // Debug Print CMD

	bytes_to_send[6] = (data_length >> 8);
	bytes_to_send[7] = data_length & 0xFF;

	//Copy the data
	memcpy(&bytes_to_send[8], data, data_length);
//	HAL_UART_Transmit(&huart3, bytes_to_send, 8 + data_length, 100);
}

struct Sensniff_command* sensniff_receive() {
	// Read the first 5 bytes to get magic bytes and the command
	uint8_t uart_rx_buf[12];
	memset(uart_rx_buf, 0, 12 * sizeof(uint8_t));

	HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, uart_rx_buf, 8, 100);

	if (status != HAL_OK) {
		return NULL;
	}

	//Compare magic header
	if (memcmp(sensniff_bytes, uart_rx_buf, 4) != 0) {
		// Not sensniff
		return NULL;
	}

	struct Sensniff_command* command = malloc(sizeof(struct Sensniff_command));
	command->magic_header = (uint32_t) uart_rx_buf[0];
	command->version = uart_rx_buf[4];
	command->cmd = uart_rx_buf[5];
	command->data_length = (uint16_t) uart_rx_buf[6];

	if (command->data_length > 4) {
		//Not supported
		return NULL;
	}

	if (command->data_length == 0) {
		return command;
	}

	status = HAL_UART_Receive(&huart3, uart_rx_buf, command->data_length, 100);

	if (status != HAL_OK) {
		//Failed
		return NULL;
	}

	command->data = &uart_rx_buf[7];

	return command;

}
