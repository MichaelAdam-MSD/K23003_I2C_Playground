/**
 * 	@file		mcp4716.c
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Driver for the MCP4716 DAC
 *  
 */

// Includes
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mcp4716.h"
#include "main.h"

// Defines
#define MCP4716_I2C_HANDLER		hi2c2

// Enumerators

// Variables
extern I2C_HandleTypeDef MCP4716_I2C_HANDLER;

// Local Function Prototypes

// Functions
ErrorStatus MCP4716_Shutdown(uint8_t device_addr)
{
	uint8_t buf[5];

}

void MCP4716_Write_DAC(uint8_t device_addr, uint16_t value)
{
	uint8_t buf[5];

	buf[0] = ((uint8_t) ((value>>4) & 0xFF));
	buf[1] = ((uint8_t) ((value<<4) & 0xF0));

	HAL_I2C_Master_Transmit(&MCP4716_I2C_HANDLER, device_addr, buf, 2, HAL_MAX_DELAY);
}
