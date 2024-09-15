/**
 * 	@file		mcp4716.h
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Driver for the MCP4716 DAC
 *  
 */

#ifndef MCP4716_STM32
#define MCP4716_STM32

// Includes
#include <stdbool.h>
#include "main.h"

// Defines
// Addresses
#define 	MCP4716_ADDR_0			0x60 << 1
#define 	MCP4716_ADDR_1			0x61 << 1
#define 	MCP4716_ADDR_2			0x62 << 1
#define 	MCP4716_ADDR_3			0x63 << 1
#define 	MCP4716_ADDR_4			0x64 << 1
#define 	MCP4716_ADDR_5			0x65 << 1
#define 	MCP4716_ADDR_6			0x66 << 1
#define 	MCP4716_ADDR_7			0x67 << 1

// Registers

// Enumerators

// Function Prototypes
ErrorStatus MCP4716_Init(uint8_t device_addr);

void MCP4716_Write_DAC(uint8_t device_addr, uint16_t value);

#endif //MCP4716_STM32
