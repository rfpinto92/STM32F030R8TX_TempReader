/*
 * AHT10.c
 *
 *  Created on: Jul 13, 2022
 *      Author: ricar
 */
static const uint8_tn AHT10_sensAddress=0x38 << 1; //Use 8-bit address
static const uint8_tn AHT10_InitComand=0xE1 << 1; //Use 8-bit address
static const uint8_tn AHT10_ReadTempComand=0xAC << 1; //Use 8-bit address


HAL_StatusTypeDef AHT10_I2cStatus;
/*
 *
 *
 *
 *
 */
uint8_t Init_AHT10();






