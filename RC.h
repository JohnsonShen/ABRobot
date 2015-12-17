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
#include "Def.h"
#ifndef _RC_H
#define _RC_H
#define THROTTLEPIN      0  
#define ROLLPIN          1  
#define PITCHPIN         2  
#define YAWPIN           3  
#define AUX1PIN          4  
#define AUX2PIN          5  

#define RC_ROLL_MIN      1090
#define RC_ROLL_MID      1500
#define RC_ROLL_MAX      1920
#define RC_PITCH_MIN     1090
#define RC_PITCH_MID     1500
#define RC_PITCH_MAX     1920

#define RC_YAW_MIN       1090
#define RC_YAW_MID       1500
#define RC_YAW_MAX       1920
#define RC_THR_MIN       1090
#define RC_THR_ARM       1100
#ifdef ABROBOT
#define RC_THR_MID       250
#else
#define RC_THR_MID       1500
#endif
#define RC_THR_MAX       1920
#define RC_THR_HOV       1355
#define RC_ALT_MIN       1090
#define RC_ALT_MID       1511
#define RC_ALT_MAX       1920

#define RC_MIN_CMD       1000
#define RC_MIN_CHECK     1100
#define RC_MAX_CHECK     1900
#define RC_ONE_THIRD     1333
#define RC_TWO_THIRD     1666
#define RC_ALT_ON        1778
#define RC_ALT_OFF       1178
#define RC_ARM_MAX_CHECK 1700
#define RC_ARM_MIN_CHECK 1280

#define RC_ALT_DEAD_BAND 100
#define RC_YAW_DEAD_BAND 50
#ifdef ABROBOT
#define RC_ROLL_DEAD_BAND 10
#define RC_PITCH_DEAD_BAND 10
#else
#define RC_ROLL_DEAD_BAND 0
#define RC_PITCH_DEAD_BAND 0
#endif

#define ROLL_CH          0
#define PITCH_CH         1
#define YAW_CH           2
#define THR_CH           3
#define AUX1_CH          4
#define AUX2_CH          5

#define ARM_DETECT_LATENCY 1000//ms
void RC_Init(void);
void computeRC(void);
void getRC(int16_t* rc);
uint16_t* getValue(void);
void armDetect(void);
bool checkArm(void);
void RC_Enable(char enable);
void UpdateLED(void);
void led_arm_state(char state);
void led_mag_state(char state);
void UpdateLED(void);
bool IsRCConnected(void);
void MotorDisArm(void);
bool IsSSVConnected(void);
int16_t GetRCThrust(void);
int16_t GetRCAltitude(void);
#endif
