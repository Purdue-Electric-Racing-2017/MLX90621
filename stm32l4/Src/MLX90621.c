/*
 * MLX90621.c
 *
 *  Created on: Jan 19, 2018
 *      Author: locch
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_def.h"
#include <stdlib.h>
#include <string.h>
#include "MLX90621.h"
#include <stdbool.h>

#define MLXPOLL_DELAY 			100
#define MLX_I2C_WRITE_EEPROM 	0xA0
#define MLX_I2C_READ_EEPROM 	0xA1
#define MLX_I2C_READ_RAM		0xC1
#define MLX_I2C_WRITE_RAM		0xC0
#define WRITE_OSC_TRIM			0x04
#define WRITE_CONFIG_REG		0x03
#define READ_CONFIG_REG			0x02
#define MLX_READ_RAM_CMND		0x02
#define CONFIG_REG_ADDR			0x92
#define NUM_PIXELS				64
#define POR_FLAG_MASK			0x800
#define READ_WHOLE_EEPROM 		0x0
#define MLX_I2C_RAM_SA_W  		0xC0
#define MLX_I2C_RAM_SA_R		0xC1
#define MLX_ARRAY_START			0x00
#define MLX_STEP_SIZE			0x01
#define MLX_NUM_READS			0x40
#define MLX_MEM_ADD_SIZE 		(0x00000001U)
#define PTAT_START_ADDR			0x40
#define PTAT_STEP_SIZE			0x00
#define PTAT_NUM_READS			0x01
#define SIZE_SEND				4
/**
 * \brief this function reads from the melexis sensor by polling
 * \param MLXHandle_t * mlx pointer to the melexis handle
 * \param uint16_t cmd the command required to get the correct data
 * \param uint16_t slave_addr determines if we are accessing EEPROM or RAM and R/W access
 * \param uint16_t * dest destination for read data
 * \retval HAL_Status
 */
HAL_StatusTypeDef MLX_Read(MLXHandle_t * mlx, uint16_t cmd, uint16_t slave_addr,
		uint8_t * dest, uint16_t destSize)
{
	 return HAL_I2C_Mem_Read(mlx->i2c, slave_addr, cmd, sizeof(cmd), dest, destSize, MLXPOLL_DELAY);
}

/**
 * \brief this function writes to the melexis sensor by polling
 * it is a wrapper for HAL_I2C_Mem_Write
 * \param MLXHandle_t * mlx pointer to the melexis handle
 * \param uint16_t cmd the command required to write the correct data
 * \param uint16_t  * src data to write
 * \param uint16_t srcSize size of data to write
 * \retval HAL_Status
 */
HAL_StatusTypeDef MLX_WriteRAM(MLXHandle_t * mlx, uint16_t slave_addr,
		uint8_t * src, uint16_t srcSize)
{
	return HAL_I2C_Master_Transmit(mlx->i2c, MLX_I2C_WRITE_RAM, src, srcSize, MLXPOLL_DELAY);
}

HAL_StatusTypeDef MLX_ReadRAM(MLXHandle_t * mlx, uint16_t slave_addr,
		uint8_t * dat_buff, uint16_t bufSize)
{
	return HAL_I2C_Master_Receive(mlx->i2c, MLX_I2C_READ_RAM, dat_buff, bufSize, MLXPOLL_DELAY);
}

/**
 * \brief this function initializes the mlx deveice and stores
 * its EEPROM values in the mlx handle
 * \param I2C_HandleTypeDef * hi2c pointer to the i2c handle
 * \retval mlx handle
 */
MLXHandle_t * MLX_Init(I2C_HandleTypeDef * hi2c)
{
	uint8_t packet[5];
	//Generate mlx handle
	MLXHandle_t * mlx = malloc(sizeof(*mlx));
	if (NULL == mlx) {return NULL;}
	mlx->i2c = hi2c;

	//Read whole EEPROM
	uint8_t tempEEPROM[256];
	if (HAL_OK !=
			MLX_Read(mlx, READ_WHOLE_EEPROM, MLX_I2C_READ_EEPROM, tempEEPROM, sizeof(*tempEEPROM*265)))
	{
		free(mlx);
		return NULL;
	}

	//Sort EEPROM
	memcpy(mlx->delA, &tempEEPROM, sizeof(*mlx->delA)*NUM_PIXELS);
	memcpy(mlx->TaDep, &tempEEPROM[0x40], sizeof(*mlx->TaDep)*NUM_PIXELS);
	memcpy(mlx->delAlpha, &tempEEPROM[0x80], sizeof(*mlx->delAlpha)*NUM_PIXELS);
	mlx->Ks_scale = tempEEPROM[0xC0];
	mlx->Ks4_EE = tempEEPROM[0xC4];
	mlx->AcommH = tempEEPROM[0xD0];
	mlx->AcommL = tempEEPROM[0xD1];
	mlx->AcpL = tempEEPROM[0xD3];
	mlx->AcpH = tempEEPROM[0xD4];
	mlx->Bcp = tempEEPROM[0xD5];
	mlx->alphaCPL = tempEEPROM[0xD6];
	mlx->alphaCPH = tempEEPROM[0xD7];
	mlx->tgc = tempEEPROM[0xD8];
	mlx->delAlphaScale = tempEEPROM[0xD9];
	mlx->Bi_scale = tempEEPROM[0xD9];
	mlx->alpha0L = tempEEPROM[0xE0];
	mlx->alpha0H = tempEEPROM[0xE1];
	mlx->alpha0Scale = tempEEPROM[0xE2];
	mlx->delAlphaScale = tempEEPROM[0xE3];
	mlx->epsilL = tempEEPROM[0xE4];
	mlx->epsilH = tempEEPROM[0xE5];
	mlx->KsTaL = tempEEPROM[0xE6];
	mlx->KsTaH = tempEEPROM[0xE7];
	memcpy(&(mlx->config), &tempEEPROM[0xF5], sizeof(mlx->config));
	mlx->OSCtrim = tempEEPROM[0xF7];
	//Write Oscillator Trim
	//Packet is generated according to pg. 10 of datasheet
	packet[0] = WRITE_OSC_TRIM;
	packet[1] = mlx->OSCtrim-0xAA;
	packet[2] = mlx->OSCtrim;
	packet[3] = 0x56;
	packet[4] = 0;
	if (HAL_OK != MLX_WriteRAM(mlx, MLX_I2C_WRITE_RAM, packet,
			sizeof(*packet)*4)) {
		free(mlx);
		return NULL;
	}
	//Write Config Value + POR Flag
	packet[0] = WRITE_CONFIG_REG;
	packet[1] = tempEEPROM[0xF5] - 0x55;
	packet[2] = tempEEPROM[0xF5];
	packet[3] = tempEEPROM[0xF6] - 0x55;
	packet[4] = tempEEPROM[0xF6];
	if (HAL_OK != MLX_WriteRAM(mlx, MLX_I2C_WRITE_RAM, packet, sizeof(*packet)*4)) {
			free(mlx);
			return NULL;
	}
	free(mlx);
	return mlx;

}

HAL_StatusTypeDef MLX_Read_IT(MLXHandle_t * mlx)
{
	// READ CONFIG REGISTER
	/*uint8_t por_check[2];
	uint8_t por_flag[4] = {MLX_READ_RAM_CMND, CONFIG_REG_ADDR, MLX_ARRAY_START, PTAT_NUM_READS};
	if(HAL_I2C_Master_Sequential_Transmit_IT(mlx->i2c, MLX_I2C_WRITE_RAM, por_flag, SIZE_SEND, I2C_FIRST_FRAME) == HAL_OK) {
		if(HAL_I2C_Master_Sequential_Receive_IT(mlx->i2c, MLX_I2C_READ_RAM, por_check, 2, I2C_FIRST_FRAME) != HAL_OK) {
			return HAL_BUSY;
		} else {
			printf("0:%d, 1:%d", por_check[0], por_check[1]);
			return HAL_OK;
		}
	} else {
		return HAL_BUSY;
	}
	// CHECK THAT POR FLAG IS NOT CLEARED
	if((por_check[1] & 0x04) == 0) {
		MLX_Init(mlx->i2c);
		return HAL_BUSY;
	}*/

	// READ SENSOR DATA
	uint8_t ptatdata[4] = {MLX_READ_RAM_CMND, PTAT_START_ADDR, PTAT_STEP_SIZE, PTAT_NUM_READS};
	uint8_t pdata[4] = {MLX_READ_RAM_CMND, MLX_ARRAY_START, MLX_STEP_SIZE, MLX_NUM_READS};
	uint16_t SizeSend = 4;
	uint16_t SizeReceive = 128;
	uint16_t PtatReceive = 2;

	// Sensor Data (Temperature / PTAT)
	if(HAL_I2C_Master_Sequential_Transmit_IT(mlx->i2c, MLX_I2C_RAM_SA_W, ptatdata,
		SizeSend, I2C_FIRST_FRAME) == HAL_OK) {
	if(HAL_I2C_Master_Sequential_Receive_IT(mlx->i2c, MLX_I2C_RAM_SA_R, mlx->ptat,
		PtatReceive, I2C_FIRST_FRAME) != HAL_OK) {
			return HAL_BUSY;
		}
	}

	// Tire Data (Temperature)
	if(HAL_I2C_Master_Sequential_Transmit_IT(mlx->i2c, MLX_I2C_RAM_SA_W, pdata,
		SizeSend, I2C_FIRST_FRAME) == HAL_OK) {
		return HAL_I2C_Master_Sequential_Receive_IT(mlx->i2c, MLX_I2C_RAM_SA_R, mlx->rawIR,
		SizeReceive, I2C_FIRST_FRAME);
	}
	return HAL_BUSY;
}


