/*
 * MLX90621.c

 *
 *  Created on: Jan 19, 2018
 *      Author: locch
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "MLX90621.h"
#define MLXPOLL_DELAY 100
#define MLX_I2C_WRITE_EEPROM 	0xA0
#define MLX_I2C_READ_EEPROM 	0xA1
#define MLX_I2C_READ_RAM		0xC1
#define MLX_I2C_WRITE_RAM		0xC0
#define WRITE_OSC_TRIM			0x04
#define WRITE_CONFIG_REG		0x03
#define READ_CONFIG_REG			0x02
#define CONFIG_REG_ADDR			0x92
#define NUM_PIXELS				64
#define POR_FLAG_MASK			0x800
#define READ_WHOLE_EEPROM 		0x0
#define CONFIG_MASK				0x3
#define EEPROM_MASK				0xF
#define ABSOLUTE_TEMP           273.15
#define TWO_BYTE_MAX 		    32768

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
/* This function is supposed to Initiate a read request to mlx device*/
/*void MLX_Read_IT(MLXHandle * mlx){
	//Check POR/Brownout Flag
	uint16_t tempConfigReg;
	MLX_Read(mlx, READ_CONFIG_REG, &tempConfigReg, sizeof)
	//Initiate data request

}*/

/**
 * \brief this function initializes the mlx device and stores
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
			MLX_Read(mlx,READ_WHOLE_EEPROM, MLX_I2C_READ_EEPROM, tempEEPROM, sizeof(*tempEEPROM*256)))
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
	////// Added by: Aninda Saha ///////
	memcpy(&(mlx->Vth), &tempEEPROM[0xDA], sizeof(mlx->Vth));
	memcpy(&(mlx->Kt1), &tempEEPROM[0xDC], sizeof(mlx->Kt1));
	memcpy(&(mlx->Kt2), &tempEEPROM[0xDE], sizeof(mlx->Kt2));
	mlx->Kt_scale = tempEEPROM[0xD2];
	///////////////////////////////
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
		if (HAL_OK != MLX_WriteRAM(mlx, MLX_I2C_WRITE_RAM, packet,
				sizeof(*packet)*4)) {
			free(mlx);
			return NULL;
		}
	free(mlx);
	return mlx;

}

//void MLX_DeInit()

/**
 * \brief - this function calculates the absolute chip temperatures
 *
 * \param - MLXHandle_t* mlx - the pointer to the melexis handle
 *
 * \retval - (double) calculated absolute chip temperature
 */

double Calc_Ta(MLXHandle_t* mlx) {

	// Ta = (-Kt1 + sqrt(power(Kt1, 2)-4*Kt2*(Vth-PTAT_DATA)))/(2*Kt2) + 25

	// Calculating Vth
	uint8_t config_reg = (uint8_t) ((mlx -> config) >> 4) & CONFIG_MASK;
	(mlx -> Vth) = (mlx -> Vth) / pow(2, 3-config_reg);

	// Calculating Kt1
	uint8_t Kt_scale_shift1 = (uint8_t) ((mlx -> Kt_scale) >> 4) & EEPROM_MASK;
	(mlx -> Kt1) = (mlx -> Kt1) / (pow(2, Kt_scale_shift1) * pow(2, 3-config_reg));

	// Calculating Kt2
	uint8_t Kt_scale_shift2 = (uint8_t) (mlx -> Kt_scale) & EEPROM_MASK;
	(mlx -> Kt2) = (mlx -> Kt2) / (pow(2, Kt_scale_shift2+10) * pow(2, 3-config_reg));

	// Extracting PTAT_Data
	uint16_t PTAT_data = (mlx->rawIR)[64];

	// Calculating Ta
	double Ta = (-(mlx->Kt1)+sqrt(pow(mlx->Kt1,2))-4*(mlx->Kt2)*((mlx->Vth-PTAT_data)))
																  /(2*(mlx->Kt2)) + 25;
	return Ta;

}

/**
 * \brief - the parasitic free IR compensated signal
 *
 * \param int8_t i - the row value of the pixel being observed
 * \param int8_t j - the column value of the pixel being observed
 * \param double Ta - absolute chip temperature as calculated previously
 *
 * \retval - (double) Vir compensated value
 */

double Calc_Vir_Compensated(MLXHandle_t* mlx, int8_t i, int8_t j, double Ta) {

	// Vir Offset Compensation
	double Ai, Bi;
	uint8_t address = i + 4*j;
	int8_t Ta0 = 25;
	uint8_t config_reg = (uint8_t) ((mlx->config)>>4) & CONFIG_MASK;

	uint16_t Vir = (mlx->rawIR)[address];

	int16_t Acomm = 256*(mlx->AcommH) + (mlx->AcommL);
	uint8_t delAij = (mlx->delA)[address];
	uint8_t delA_scale = (uint8_t) ((mlx->delAlphaScale)>>4) & EEPROM_MASK;
	Ai = (Acomm + delAij*pow(2, delA_scale))/pow(2, 3-config_reg);

	int8_t Bij = (mlx->TaDep)[address];
	uint8_t Bi_scale = (uint8_t) (mlx->Bi_scale) & EEPROM_MASK;
	Bi = Bij/(pow(2, Bi_scale)*pow(2, 3-config_reg));

	double Vir_Offcompensated = Vir - (Ai + Bi*(Ta-Ta0));

	// Vir Thermal Gradient Compensation
	uint16_t Acp = 256*(mlx->AcpH) + (mlx->AcpL);
	double VirCP_Offcompensated = Acp - (Ai + Bi*(Ta-25));
	double tgc = (mlx->tgc)/32;

	double VirTGC_Compensated = Vir_Offcompensated - tgc*VirCP_Offcompensated;

	// Emissivity compensation
	double epsil = (256*(mlx->epsilH)+(mlx->epsilL))/TWO_BYTE_MAX;

	double Vir_Compensated = VirTGC_Compensated/epsil;

	return Vir_Compensated;

}

/**
 * \brief - this function calculates the Sensitivity to Dependence (slope) by reading
 * 			in Ks4_EE value from EEPROM and scaling it appropriately
 *
 * \param - MLXHandle_t* mlx - the pointer to the melexis handle
 * \param int8_t i - the row value of the pixel being observed
 * \param int8_t j - the column value of the pixel being observed
 * \param double Ta - absolute chip temperature as calculated previously
 *
 * \retval (double) alpha compensated value
 */

double Calc_Alpha_Compensated(MLXHandle_t* mlx, int8_t i, int8_t j, double Ta) {

	// alpha_comp = (1 + KsTa*(Ta-Ta0))*(alpha_ij-tgc*alpha_cp);

	uint8_t TA0 = 25;
	double KsTa = (256*(mlx->KsTaH)+(mlx->KsTaL))/pow(2,20);

	// Calculating alpha_ij
	uint8_t config_reg = (uint8_t) ((mlx -> config) >> 4) & CONFIG_MASK;
	double alpha0 = (256*(mlx->alpha0H)+(mlx->alpha0L))/pow(2, (mlx->alpha0Scale));
	uint8_t address = i + 4*j;
	double delAlpha = ((mlx->delA)[address])/pow(2,(mlx->delAlphaScale));
	double alpha_ij = (alpha0 + delAlpha)/pow(2, 3-config_reg);

	// Calculating alpha_cp
	double alpha_cp = (256*(mlx->alphaCPH)+(mlx->alphaCPL))/(pow(2,(mlx->alpha0Scale))*
																pow(2, 3-config_reg));

	double alpha_comp = (1 + KsTa*(Ta-TA0))*(alpha_ij-(mlx->tgc)*alpha_cp);

	return alpha_comp;
}
/**
 * \brief - this function calculates the Sensitivity to Dependence (slope) by reading
 * 			in Ks4_EE value from EEPROM and scaling it appropriately
 *
 * \param - MLXHandle_t* mlx - the pointer to the melexis handle
 *
 * \retval - (double) adjusted Ks4_EE value
 */

double Calc_Ks4(MLXHandle_t* mlx) {

	double Ks4 = (mlx->Ks4_EE)/pow(2, (mlx->Ks_scale)+8);
	return Ks4;
}

/**
 * \brief this function is responsible for calculating the processed temperature
 * 		  value. the function calls multiple other subroutines to calculate the
 * 		  necessary parameters
 *
 * \param MLXHandle_t* mlx - a pointer to the melexis handle
 * \param int8_t i - the row value of the pixel being observed
 * \param int8_t j - the column value of the pixel being observed
 *
 * \retval - (double) processed observed temperature
 */

double MLX_CalcTemp(MLXHandle_t* mlx, int8_t i, int8_t j) {

	// Do a check for the range of i and j
	if(!(i<4 && j<16)) { exit(EXIT_FAILURE); }

	// Calculating all necessary parameters
	double Ta = Calc_Ta(mlx);
	double Vir_Compensated = Calc_Vir_Compensated(mlx, i, j, Ta);
	double Alpha_Compensated = Calc_Alpha_Compensated(mlx, i, j, Ta);
	double Ks4 = Calc_Ks4(mlx);
	double TaK4 = pow(Ta+ABSOLUTE_TEMP, 4);

	// Calculating Sx
	double tmp1 = pow(Alpha_Compensated, 3)*Vir_Compensated;
	double tmp2 = pow(Alpha_Compensated, 4)*TaK4;
	double Sx = Ks4*pow(tmp1+tmp2, 1/4);

	// Calculating T0
	double tmp3 = Alpha_Compensated*(1-Ks4*ABSOLUTE_TEMP) + Sx;
	double t0 = pow((Vir_Compensated/tmp3)+TaK4, 1/4) - ABSOLUTE_TEMP;

	return t0;
}


