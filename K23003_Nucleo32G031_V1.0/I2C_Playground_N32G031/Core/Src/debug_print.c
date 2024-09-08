/*
 * debug_print.c
 *
 *  Created on: Jan 4, 2024
 *      Author: micha
 */

#include "debug_print.h"

extern UART_HandleTypeDef huart2;

char tx_buf[TX_BUF_SIZE];

// Use this instead of printf()
void debug_print(char *format, ...) {
	va_list args;
	va_start(args, format);

	uint16_t size = vsnprintf(tx_buf, TX_BUF_SIZE, format, args);

	//project_print_user_implementation(tx_buf, size);

	HAL_UART_Transmit(&huart2, (uint8_t*) tx_buf, size, 100);
}

