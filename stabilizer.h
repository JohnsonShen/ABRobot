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
#ifndef STABILIZER_H_
#define STABILIZER_H_
#include <stdint.h>
void stabilizer(void);
void stabilizerInit(void);
void GetMotorPower(uint16_t* MotorPower);
char GetFlip(void);
void ClearFlip(void);
float GetAccWZ(void);
void GetvSpeed(float* speed);
void SetvSpeed(float speed);
float deadband(float value, const float threshold);
uint16_t limitThrust(int32_t value);
float GetactuatorThrust(void);
void HoldHead(void);
void HoldHeadFree(void);
bool getMagMode(void);
bool getHeadFreeMode(void);
bool isArmMinThrottle(void);
bool IsMotorSpin(void);
#endif /* STABILIZER_H_ */

