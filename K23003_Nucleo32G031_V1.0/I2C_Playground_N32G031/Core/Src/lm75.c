/**
 * 	@file		lm75.c
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
#include "lm75.h"
#include "main.h"

// Defines
#define LM75_I2C_HANDLER		hi2c2

// Enumerators

// Variables
extern I2C_HandleTypeDef LM75_I2C_HANDLER;

// Local Function Prototypes

// Functions
ErrorStatus LM75_Shutdown(uint8_t shutdown, uint8_t device_addr) //0 = Active, 1 = Shutdown
{
	uint8_t buf[5];

	buf[0] = LM75_CONFIG_REG;
	if (shutdown)
	{
		buf[1] = LM75_SHDN;
	}
	else
	{
		buf[1] = LM75_ACTIVE;
	}

	if (HAL_I2C_Master_Transmit(&LM75_I2C_HANDLER, device_addr, buf, strlen((char*) buf), 1) != HAL_OK)
		return ERROR;

	return SUCCESS;
}

ErrorStatus LM75_Init(uint8_t device_addr)
{
	if (HAL_I2C_IsDeviceReady(&LM75_I2C_HANDLER, device_addr, 3, 20000) != HAL_OK)
	{
		// Return error
		return ERROR;
	}

	return SUCCESS;
}

float LM75_Poll_Temperature(uint8_t device_addr)
{
	uint8_t buf[5];
	int16_t raw_temperature;
	uint8_t negative;
	float temperature;

	buf[0] = LM75_TEMP_REG;
	HAL_I2C_Master_Transmit(&LM75_I2C_HANDLER, device_addr, buf, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&LM75_I2C_HANDLER, device_addr, buf, 2, HAL_MAX_DELAY);

	negative = (buf[0] & 0x80) ? 1 : 0;
	raw_temperature = (buf[0] << 3) | buf[1] >> 5;

	if ((raw_temperature & 0x0400) != 0) /* check first bit */
	{
		raw_temperature = (raw_temperature) | 0xF800U; /* set negative part */
		temperature = (float) (-(~raw_temperature + 1)) * 0.125f; /* if negative set convert temp */
	}
	else
	{
		temperature = (float) raw_temperature * 0.125f; /* if positive set convert temp */
	}

	return temperature;
}
