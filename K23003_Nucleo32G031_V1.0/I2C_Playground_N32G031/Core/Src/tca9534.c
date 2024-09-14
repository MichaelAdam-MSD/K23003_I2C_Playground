/**
 * 	@file		tca9534.c
 *
 *	@author		Michael Adam <michael.adam5@proton.me>
 *	@date		2024-09-14
 *
 *  @brief		Library for the TCA9534 / TCA9534A IO expander
 *  
 */

// Includes
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tca9534.h"
#include "main.h"

// Defines
#define TCA9534_I2C_HANDLER		hi2c2

// Enumerators

// Variables
extern I2C_HandleTypeDef TCA9534_I2C_HANDLER;

// Local Function Prototypes
void TCA9534_Write_Output_Reg(uint8_t device_addr, uint8_t byte);

uint8_t TCA9534_Read_Input_Reg(uint8_t device_addr);

uint32_t bit_config(uint32_t word, uint32_t bit_number, uint32_t bit_value);

// Functions
ErrorStatus TCA9534_Init(uint8_t device_addr, uint8_t direction)
{
	uint8_t buf[2];

	if (HAL_I2C_IsDeviceReady(&TCA9534_I2C_HANDLER, device_addr, 3, 20000) != HAL_OK)
	{
		// Return error
		return ERROR;
	}

	buf[0] = TCA9534_CONFIG_REG;
	buf[1] = direction;

	HAL_I2C_Master_Transmit(&TCA9534_I2C_HANDLER, device_addr, buf, 2, HAL_MAX_DELAY);

	return SUCCESS;
}

void TCA9534_Write_Output_Reg(uint8_t device_addr, uint8_t byte)
{
	uint8_t buf[2];

	buf[0] = TCA9534_OUTPUT_REG;
	buf[1] = byte;

	HAL_I2C_Master_Transmit(&TCA9534_I2C_HANDLER, device_addr, buf, 2, HAL_MAX_DELAY);
}

uint8_t TCA9534_Read_Input_Reg(uint8_t device_addr)
{
	uint8_t buf[2];

	buf[0] = TCA9534_INPUT_REG;

	HAL_I2C_Master_Transmit(&TCA9534_I2C_HANDLER, device_addr, buf, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&TCA9534_I2C_HANDLER, device_addr, buf, 1, HAL_MAX_DELAY);

	return buf[0];
}

uint8_t TCA9534_Read_Output_Reg(uint8_t device_addr)
{
	uint8_t buf[2];

	buf[0] = TCA9534_OUTPUT_REG;

	HAL_I2C_Master_Transmit(&TCA9534_I2C_HANDLER, device_addr, buf, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&TCA9534_I2C_HANDLER, device_addr, buf, 1, HAL_MAX_DELAY);

	return buf[0];
}

// #########################################################
// Board specific code
// #########################################################
uint8_t Read_PB_P4(void)
{
	uint8_t read_byte =	TCA9534_Read_Input_Reg(TCA9534A_ADDR_0);

	read_byte = read_byte & 0b00010000;

	return read_byte;
}

uint8_t Read_PB_P5(void)
{
	uint8_t read_byte =	TCA9534_Read_Input_Reg(TCA9534A_ADDR_0);

	read_byte = read_byte & 0b00100000;

	return read_byte;
}

uint8_t Read_PB_P6(void)
{
	uint8_t read_byte =	TCA9534_Read_Input_Reg(TCA9534A_ADDR_0);

	read_byte = read_byte & 0b01000000;

	return read_byte;
}

uint8_t Read_PB_P7(void)
{
	uint8_t read_byte =	TCA9534_Read_Input_Reg(TCA9534A_ADDR_0);

	read_byte = read_byte & 0b10000000;

	return read_byte;
}

uint8_t Read_INT(void)
{
	return HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin);
}

void Write_LED(uint8_t state, uint8_t led_number)
{
	uint8_t reg_value = TCA9534_Read_Output_Reg(TCA9534A_ADDR_0);

	reg_value = bit_config(reg_value, led_number,state);

	TCA9534_Write_Output_Reg(TCA9534A_ADDR_0, reg_value);
}

uint32_t bit_config(uint32_t word, uint32_t bit_number, uint32_t bit_value)
{
    uint32_t word_temp = word;
    word_temp ^= (-bit_value ^ word_temp) & (1 << bit_number);
    return word_temp;
}
