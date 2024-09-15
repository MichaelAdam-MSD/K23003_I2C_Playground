/**
 * 	@file		lm75.h
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Driver for the LM75 Temperature sensor
 *  
 */

#ifndef MCP3021_STM32
#define MCP3021_STM32

// Includes
#include <stdbool.h>
#include "main.h"

// Defines
// Addresses
#define 	MCP3021_ADDR			0x4D << 1


// Registers

// Enumerators

// Function Prototypes
ErrorStatus MCP3021_Init(uint8_t device_addr);

uint16_t MCP3021_Read_ADC_Counts(uint8_t device_addr);

#endif //MCP3021_STM32
