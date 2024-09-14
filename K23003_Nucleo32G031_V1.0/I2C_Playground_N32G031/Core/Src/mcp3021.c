/**
 * 	@file		mcp3021.c
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Driver for the LM75 Temperature sensor
 *  
 */

// Includes
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mcp3021.h"
#include "main.h"

// Defines
#define MCP3021_I2C_HANDLER		hi2c2

// Enumerators

// Variables
extern I2C_HandleTypeDef MCP3021_I2C_HANDLER;

// Local Function Prototypes

// Functions
uint16_t MCP3021_Read_ADC_Counts(uint8_t device_addr)
{
	uint8_t buf[2];
	uint16_t adc_counts;

	HAL_I2C_Master_Receive(&MCP3021_I2C_HANDLER, device_addr, buf, 2, HAL_MAX_DELAY);

	adc_counts = (buf[0] << 6) | (buf[1] >> 2);

	return adc_counts;
}
