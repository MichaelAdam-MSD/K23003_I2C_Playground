/*
 * eeprom.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Michael Adam
 */

// Includes
#include "main.h"
#include "eeprom.h"

// Defines
#define EEPROM_I2C_HANDLER			hi2c2
#define EEPROM_COM_MAX_DELAY		10

// Variables
extern I2C_HandleTypeDef EEPROM_I2C_HANDLER;

// Local Function Prototypes

// Functions
ErrorStatus M24C0X_Init(uint8_t device_addr)
{
	if (HAL_I2C_IsDeviceReady(&EEPROM_I2C_HANDLER, device_addr, 1, EEPROM_COM_MAX_DELAY) != HAL_OK)
	{
		/* Return false */
		return ERROR;
	}

	return SUCCESS;
}

ErrorStatus M24C0X_Write_2Byte(uint8_t device_addr, uint8_t addr, uint8_t lsb, uint8_t msb) // Processing time 380us
{
	uint8_t buf[3];

	buf[0] = addr;
	buf[1] = msb;
	buf[2] = lsb;

	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 3, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;

	return SUCCESS;
}

ErrorStatus M24C0X_Write_Byte(uint8_t device_addr, uint8_t addr, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = addr;
	buf[1] = data;

	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 2, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;

	return SUCCESS;
}

uint8_t M24C0X_Read_Byte(uint8_t device_addr, uint8_t addr)
{
	uint8_t buf[1];

	buf[0] = addr;
	HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 1, EEPROM_COM_MAX_DELAY);

	HAL_I2C_Master_Receive(&EEPROM_I2C_HANDLER, device_addr, buf, 1, EEPROM_COM_MAX_DELAY);

	return buf[0];
}

uint16_t M24C0X_Read_Word(uint8_t device_addr, uint8_t addr) // Processing time 500us
{
	uint16_t word;
	uint8_t buf[2];

	buf[0] = addr;

	HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 1, EEPROM_COM_MAX_DELAY);

	HAL_I2C_Master_Receive(&EEPROM_I2C_HANDLER, device_addr, buf, 2, EEPROM_COM_MAX_DELAY);

	word = ((uint16_t) buf[0] << 8) | buf[1];

	return word;
}

void M24C0X_Write_Word(uint8_t device_addr, uint8_t addr, uint16_t word)
{
	uint8_t buf[2];
	buf[0] = (uint8_t) (word);
	buf[1]= (uint8_t) (word >> 8);
	M24C0X_Write_2Byte(device_addr, addr, buf[0], buf[1]);
}

