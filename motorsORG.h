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
#ifndef __MOTORS_H__
#define __MOTORS_H__
#include <stdint.h>
/******** Defines ********/
#define MOTORS_PWM_BITS     9
#define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#define MOTORS_TEST_RATIO          (uint16_t)(0.5*(1<<16))
#define MOTORS_TEST_ON_TIME_MS     10
#define MOTORS_TEST_DELAY_TIME_MS  50
#define ESC_UPDATE_FREQ            400
#define MOTORS_ESC_DELAY           1500 /*Frame*/
#define MOTOR_NUMBER               4
void motorsInit(void);
void motorsSetRatio(int id, uint16_t ratio);
int motorsGetRatio(int id);
void motorsStart(void);
#endif /* __MOTORS_H__ */

