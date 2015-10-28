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
#ifndef _FLASH_CTRL_H
#define _FLASH_CTRL_H
#include "Def.h"
#define BOARD_CODE_BASE 12
//Calibration Section
#define CAL_BASE	16
#define FIELD_VALID_SIZE 1
#define QUALITY_FACTOR_SIZE 1
#define FIELD_VALID      0x77
#define FIELD_INVALID    0x88
#define CAL_BASE_GYRO	 CAL_BASE
#define CAL_BASE_ACC	(CAL_BASE_GYRO + GYRO_CAL_DATA_SIZE + FIELD_VALID_SIZE)
#define CAL_BASE_MAG	(CAL_BASE_ACC + ACC_CAL_DATA_SIZE + FIELD_VALID_SIZE)

//PID Section
#define PID_SIZE 3
#define ALT_PID_SIZE 1
#define PPID_DATA_SIZE 9
#define RPID_DATA_SIZE 9
#define APID_DATA_SIZE 3
#define PID_FIELD_SIZE 		(PPID_DATA_SIZE + RPID_DATA_SIZE + APID_DATA_SIZE)
#define PID_BASE (CAL_BASE_MAG + MAG_CAL_DATA_SIZE + FIELD_VALID_SIZE + QUALITY_FACTOR_SIZE)

//SSVRC Section
#define RX_ADDRESS_SIZE 2
#define RX_ADDRESS_BASE (PID_BASE + PID_FIELD_SIZE + FIELD_VALID_SIZE)

void FlashInit(void);
void UpdateBoardVersion(bool);
uint32_t GetBoardVersion(void);
void UpdateFlashCal(int8_t sensorType, bool erase);
bool GetFlashCal(int8_t sensorType, float* Cal);
void UpdateFlashPID(bool erase);
bool GetFlashPID(float* PID_FIELD);
void UpdateFlashRxAddress(void);
bool GetFlashRxAddress(uint8_t* RX_ADDRESS_FIELD);
void TestFloat(void);
float GetFloatCounter(void);
int32_t float2dw(float f);
float dw2float(int32_t dw);
void FlashControl(void);
#endif
