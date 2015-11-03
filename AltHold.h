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
#ifndef _ALTHOLD_H
#define _ALTHOLD_H
#include "pid.h"
typedef struct ALT_STATE {
	float altHoldPIDVal;
	float altHoldChange;
	float altHoldTarget;
	float asl;
}ALTHOLD_STATE_T;
bool GetAltHoldMode(void);
void stabilizerAltHoldUpdate(int16_t *);
bool GetAltHoldMode(void);
void SetAltHoldPIDObj(PidObject* PIDObj);
void SetCalibratingB(uint8_t c);
ALTHOLD_STATE_T* GetAltHoldState(void);
float getAslSpeed(void);
float getAltHoldPIDVal(void);
float getAccFuse(void);
bool GetAutoLandMode(void);
bool GetNearLand(void);
#endif
