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
#ifndef _RC_SSV_H
#define _RC_SSV_H
void ssv_rc_update(void);
bool RC_SSV_Init(void);
void RC_SSV_Enable(char enable);
void SysTick_SSV(void);
void RC_CheckFlyMode(void);
uint8_t  RC_GetFlyMode(void);
uint16_t readSsvRC(uint8_t chan) ;
bool IsSSVRCConnected(void);
bool IsSSVReceiverConnected(void);
int16_t GetSSVThrust(void);
int16_t GetSSVAltitude(void);
#endif
