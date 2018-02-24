/*
 * MLX90621.h
 *
 *  Created on: Jan 19, 2018
 *      Author: locch
 */

#ifndef MLX90621_H_
#define MLX90621_H_

typedef struct {
	I2C_HandleTypeDef * i2c;
	uint16_t rawIR[66];
	uint8_t delA[64];		///64 member array of IR pixel individual offset delta coefficient
	int8_t TaDep[64]; 	/**64 member array of Individual Ta dependence (slope)
				  	  	  	  of IR pixels offset*/
	uint8_t delAlpha[64];///64 member array of Individual sensitivity coefficients
	uint8_t Ks_scale;
	int8_t  Ks4_EE; 	///Sensitivity To dependence (slope)
	int8_t   AcommL;	///IR pixel common offset coefficient (low)
	int8_t  AcommH;		///IR pixel common offset coefficient (high)
	int8_t	AcpL;		///Compensation pixel individual offset coefficient (low)
	int8_t   AcpH;		///Compensation pixel individual offset coefficient (high)
	int8_t   Bcp;		///Individual Ta dependence (slope) of the compensation pixel offset
	uint8_t  alphaCPL;		///Sensitivity coefficient of the compensation pixel (low)
	uint8_t  alphaCPH;		///Sensitivity coefficient of the compensation pixel (high)
	int8_t   tgc;		///Thermal gradient coefficient
	uint8_t  Ai_scale;	///Scaling coeff for the IR pixels offset (delA[])
	uint8_t  Bi_scale;	///Scaling coeff of the IR pixels offset Ta dep. (TaDep[])
	int16_t Vth; // Vth0 of absolute temperature sensor
	int16_t Kt1; // Kt1 of absolute temperature sensor
	int16_t Kt2; // Kt2 of absolute temperature sensor
	int8_t Kt_scale; // First nibble is Kt1 scale, second nibble is Kt2 scale
	uint8_t  alpha0L;	///Common sensitivity coefficient of IR pixels (low)
	uint8_t  alpha0H;	///Common sensitivity coefficient of IR pixels (high)
	uint8_t  alpha0Scale; ///Scaling coefficient for common sensitivity
	uint8_t  delAlphaScale; ///Scaling coefficient for individual sensitivit
	uint8_t  epsilL; 	///Emissivity (low)
	uint8_t  epsilH;	///Emissivity (high)
	int8_t   KsTaL;		///KsTa (fixed scale coefficient = 20) (low)
	int8_t   KsTaH;		///KsTa (fixed scale coefficient = 20) (high)
	uint16_t config;	///Configuration register of mlx sensor
	uint8_t	 OSCtrim;	///Oscillator Trim
} MLXHandle_t;

MLXHandle_t * MLX_Init(I2C_HandleTypeDef * hi2c);

#endif /* MLX90621_H_ */
