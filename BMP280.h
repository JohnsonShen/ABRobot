/*============================================================================*
 * O     O          __                   ______  __                           *
 *  \   /      /\  / /_      _    __    / /___/ / /_     _                    *
 *   [+]      /  \/ / \\    //__ / /__ / /____ / / \\   //                    *
 *  /   \    / /\  /   \\__// --/ /---/ /----// /   \\_//                     *
 * O     O  /_/  \/     \__/    \_\/ /_/     /_/ ____/_/                      *
 *                                                                            *
 *                                                                            *
 * Multi-Rotor controller firmware for Nuvoton Cortex M4 series               *
 *                                                                            *
 * KevinHsu add BMP280 support [2015.3.25]                      .             *
 *                                                                            *
 *============================================================================*
 */
#ifndef BMP280_H
#define BMP280_H
#include <stdbool.h>

#define BMP280_ADDRESS 0x76 				// 7-bit address

/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG 0x88
#define BMP280_DIG_T1_MSB_REG 0x89
#define BMP280_DIG_T2_LSB_REG 0x8A
#define BMP280_DIG_T2_MSB_REG 0x8B
#define BMP280_DIG_T3_LSB_REG 0x8C
#define BMP280_DIG_T3_MSB_REG 0x8D
#define BMP280_DIG_P1_LSB_REG 0x8E
#define BMP280_DIG_P1_MSB_REG 0x8F
#define BMP280_DIG_P2_LSB_REG 0x90
#define BMP280_DIG_P2_MSB_REG 0x91
#define BMP280_DIG_P3_LSB_REG 0x92
#define BMP280_DIG_P3_MSB_REG 0x93
#define BMP280_DIG_P4_LSB_REG 0x94
#define BMP280_DIG_P4_MSB_REG 0x95
#define BMP280_DIG_P5_LSB_REG 0x96
#define BMP280_DIG_P5_MSB_REG 0x97
#define BMP280_DIG_P6_LSB_REG 0x98
#define BMP280_DIG_P6_MSB_REG 0x99
#define BMP280_DIG_P7_LSB_REG 0x9A
#define BMP280_DIG_P7_MSB_REG 0x9B
#define BMP280_DIG_P8_LSB_REG 0x9C
#define BMP280_DIG_P8_MSB_REG 0x9D
#define BMP280_DIG_P9_LSB_REG 0x9E
#define BMP280_DIG_P9_MSB_REG 0x9F

/************************************************/
/**\name REGISTER ADDRESS DEFINITION */
/***********************************************/
#define BMP280_CHIP_ID_REG 0xD0 /*Chip ID Register */
#define BMP280_RST_REG 0xE0 /*Softreset Register */
#define BMP280_STAT_REG 0xF3 /*Status Register */
#define BMP280_CTRL_MEAS_REG 0xF4 /*Ctrl Measure Register */
#define BMP280_CONFIG_REG 0xF5 /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG 0xF7 /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG 0xF8 /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG 0xF9 /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG 0xFA /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG 0xFB /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG 0xFC /*Temperature XLSB Reg */

#define	BMP280_COMMAND_TEMPERATURE 0x2E
#define	BMP280_COMMAND_PRESSURE0 0x25
#define	BMP280_COMMAND_PRESSURE1 0x29
#define	BMP280_COMMAND_PRESSURE2 0x2D
#define	BMP280_COMMAND_PRESSURE3 0x31
#define	BMP280_COMMAND_PRESSURE4 0x5D 

bool Int_BMP280(void);
bool BMP280_GetData(float* pressure, float* temperature, float* asl);
bool BMP280SelfTest(void);

#endif // BMP280_H
