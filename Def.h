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
 * Written by by T.L. Shen for Nuvoton Technology.                            *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                  *
 *                                                                            *
 *============================================================================*
 */
#ifndef DEF_H_
#define DEF_H_
#include "Config.h"
#include <stdbool.h>
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif
#ifndef bool
#define bool unsigned char
#endif

#if (BOARD_CODE == 140)
#define USE_I2C_PORT1
/************************/
/*      ACC/GYRO        */
/************************/
#define MPU6050
/************************/
/*        MAG           */
/************************/
#define HMC5883
//#define AK8975
/************************/
/*       BARO           */
/************************/
//#define BMP085 0
#define MS5611 1
#define BMP280 2
#endif
#if (BOARD_CODE == 150)
#define USE_I2C_PORT0
/************************/
/*      ACC/GYRO        */
/************************/
#define MPU6050
/************************/
/*        MAG           */
/************************/
#define HMC5883
//#define AK8975
/************************/
/*       BARO           */
/************************/
//#define BMP085 0
#define MS5611 1
#define BMP280 2
#define HEX6X
#define H300
#endif
#if (BOARD_CODE == 200)
#define USE_I2C_PORT0
/************************/
/*      ACC/GYRO        */
/************************/
#define MPU6500
/************************/
/*        MAG           */
/************************/
#define HMC5883
//#define AK8975
/************************/
/*       BARO           */
/************************/
//#define BMP085 0
#define MS5611 1
#define BMP280 2
#define HEX6X
#define H630
#endif
#ifdef GPS
#define GPS_UBLOX
#define GPS_UART
#ifdef GPS_UART
#define GPS_BAUD      9600
#endif
#endif
/****************************************************************/
/*                    Sensor Type definitions                   */
/****************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(MPU6500) || defined(NUNCHUCK)
#define STACK_ACC 1
#if defined(MPU6050) || defined(MPU6500)
#ifndef OPTION_RC
#define ACC_ORIENTATION(X, Y, Z)  {Sensor.rawACC[0] = X; Sensor.rawACC[1] = Y; Sensor.rawACC[2] = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {Sensor.rawGYRO[0] = X; Sensor.rawGYRO[1] = Y; Sensor.rawGYRO[2] = Z;}
#else
#define ACC_ORIENTATION(X, Y, Z)  {Sensor.rawACC[0] = -X; Sensor.rawACC[1] = -Y; Sensor.rawACC[2] = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {Sensor.rawGYRO[0] = -X; Sensor.rawGYRO[1] = -Y; Sensor.rawGYRO[2] = Z;}
#endif
#endif
#else
#define STACK_ACC 0
#define ACC_ORIENTATION(X, Y, Z) 
#define GYRO_ORIENTATION(X, Y, Z)
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110) || defined(AK8975)
#define STACK_MAG 1
#ifdef HMC5883
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = X; Sensor.rawMAG[1] = Y; Sensor.rawMAG[2] = Z;}
#endif
#ifdef AK8975
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = Y; Sensor.rawMAG[1] = X; Sensor.rawMAG[2] = Z;}
#endif
#else
#define STACK_MAG 0
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = X; Sensor.rawMAG[1] = Y; Sensor.rawMAG[2] = Z;}
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(MPU6500) || defined(WMP)
#define STACK_GYRO 1
#else
#define STACK_GYRO 0
#endif

#if defined(BMP085) || defined(MS5611) || defined(BMP280)
#define STACK_BARO 1
#define BARO_TAB_SIZE 21
#else
#define STACK_BARO 0
#endif
#if defined(OPTION_RC)
#define OUTPUT_RC_INTERVAL 20 //milliseconds,50Hz
#define RC_CHANS 6
#ifdef M451
#define SYSTEM_CLOCK 72000000
#else
#define SYSTEM_CLOCK 96000000
#endif
#else
#define OUTPUT_RC_INTERVAL 20 //milliseconds,50Hz
#define RC_CHANS 6
#define SYSTEM_CLOCK 72000000
#endif
#ifdef USE_I2C_PORT1
#ifdef M451
#define I2C_PORT I2C1
#else
#define I2C_PORT I2C_PORT1
#endif
#else
#ifdef M451
#define I2C_PORT I2C0
#else
#define I2C_PORT I2C_PORT0
#endif
#endif
#ifdef M451
#define IMU_UPDATE_FREQ 560
#define DATA_FLASH_TEST_BASE 0x20000
#else
#define IMU_UPDATE_FRE 300
#endif
#define ALTHOLD_UPDATE_FREQ 100
#define ALTHOLD_UPDATE_DT (float)(1.0/ALTHOLD_UPDATE_FREQ)
#define MOTOR_START_SPIN 1200
#define SYSTEM_TICK_FREQ 10000 /*10K*/
#define TICK_FRAME_PERIOD 2
#endif //DEF_H_
