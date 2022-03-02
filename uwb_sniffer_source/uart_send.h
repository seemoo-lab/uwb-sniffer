/*
 * uart_send.h
 *
 *  Created on: Mar 2, 2022
 *      Author: seemoo
 */

#ifndef UART_SEND_H_
#define UART_SEND_H_

#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <deca_types.h>

void uart_dma_send(uint8_t *bytes_to_send, int uart_len);

void log_uart(char *data);

void test_run_info(unsigned char *data);


#endif /* UART_SEND_H_ */
