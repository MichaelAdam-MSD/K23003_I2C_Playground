/**
 * 	@file		lm75.h
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Driver for the LM75 Temperature sensor
 *  
 */

#ifndef LM75_STM32
#define LM75_STM32

// Includes
#include <stdbool.h>
#include "main.h"

// Defines
// Addresses
#define 	LM75_ADDR_0			0x48 << 1
#define 	LM75_ADDR_1			0x49 << 1
#define 	LM75_ADDR_2			0x4A << 1
#define 	LM75_ADDR_3			0x4B << 1
#define 	LM75_ADDR_4			0x4C << 1
#define 	LM75_ADDR_5			0x4D << 1
#define 	LM75_ADDR_6			0x4E << 1
#define 	LM75_ADDR_7			0x4F << 1

// Registers
#define		LM75_CONFIG_REG		0x01
#define		LM75_TEMP_REG		0x00
#define		LM75_SHDN			0x01
#define 	LM75_ACTIVE			0x00

// Enumerators

// Function Prototypes
ErrorStatus LM75_Shutdown(uint8_t shutdown, uint8_t device_addr);

ErrorStatus LM75_Init(uint8_t device_addr);

float LM75_Poll_Temperature(uint8_t device_addr);

#endif //LM75_STM32
