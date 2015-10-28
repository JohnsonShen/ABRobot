 /*
* Copyright (C) 2011-2012 Bitcrazy AB
* Adapted to Cortex-M4 Fly Controller by Nuvoton
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, in version 3.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/
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
#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include "Def.h"
#ifdef HEX6X
#ifdef H630
#define RATE_KP_1806 4
#define RATE_KI_1806 0.1f
#define RATE_KD_1806 5
#endif
#ifdef H300
#define RATE_KP_1806 3
#define RATE_KI_1806 0.1f
#define RATE_KD_1806 4
#endif
#else
#define RATE_KP_1806 1.3f
#define RATE_KI_1806 0.1f
#define RATE_KD_1806 0.65f
#endif
#define LEVEL_KP_1806 3
#define LEVEL_KI_1806 1.1f
#define LEVEL_KD_1806 0
#define RATE_KP_1804 24
#define RATE_KI_1804 10.5
#define RATE_KD_1804 0
#define LEVEL_KP_1804 3.5
#define LEVEL_KI_1804 1
#define LEVEL_KD_1804 0
#define RATE_KP_2204 22.5
#define RATE_KI_2204 0.5
#define RATE_KD_2204 0.12
#define LEVEL_KP_2204 4.5
#define LEVEL_KI_2204 3
#define LEVEL_KD_2204 0
#define RATE_KP_2206 20.5
#define RATE_KI_2206 3
#define RATE_KD_2206 0.12
#define LEVEL_KP_2206 4
#define LEVEL_KI_2206 2
#define LEVEL_KD_2206 0

#define PID_ROLL_RATE_KP  RATE_KP_1806
#define PID_ROLL_RATE_KI  RATE_KI_1806
#define PID_ROLL_RATE_KD  RATE_KD_1806
#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0

#define PID_PITCH_RATE_KP  RATE_KP_1806
#define PID_PITCH_RATE_KI  RATE_KI_1806
#define PID_PITCH_RATE_KD  RATE_KD_1806
#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0
#ifdef H630
#define PID_YAW_RATE_KP  2/*20.5*/
#define PID_YAW_RATE_KI  28/*5.19f*//*200*/
#define PID_YAW_RATE_KD  0.0
#else
#define PID_YAW_RATE_KP  1.08f/*20.5*/
#define PID_YAW_RATE_KI  14/*5.19f*//*200*/
#define PID_YAW_RATE_KD  0.0
#endif
#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0
#define PID_ROLL_KP  LEVEL_KP_1806
#define PID_ROLL_KI  LEVEL_KI_1806
#define PID_ROLL_KD  LEVEL_KD_1806
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  LEVEL_KP_1806
#define PID_PITCH_KI  LEVEL_KI_1806
#define PID_PITCH_KD  LEVEL_KD_1806
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  0.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0
#define ALTHOLD_KP_BMP280 18.9
#define ALTHOLD_KI_BMP280 0.3
#define ALTHOLD_KD_BMP280 950
#define PID_ALTHOLD_KP  ALTHOLD_KP_BMP280/*36.9//13*/
#define PID_ALTHOLD_KI  ALTHOLD_KI_BMP280/*0.5*/
#ifdef LOW_WEIGHT//850mah
#define PID_ALTHOLD_KD  23//30.1
#else //1600mah
#define PID_ALTHOLD_KD ALTHOLD_KD_BMP280/*80//30.1*/
#endif

#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0
#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))
typedef struct
{
  float desired;     //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float iLimitLow;    //< integral limit
  float dt;           //< delta-time dt
} PidObject;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 */
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt);

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidSetIntegralLimit(PidObject* pid, const float limit);

void pidSetIntegralLimitLow(PidObject* pid, const float limit);
/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidReset(PidObject* pid);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 */
float pidUpdate(PidObject* pid, const float measured, const bool updateError);

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void pidSetDesired(PidObject* pid, const float desired);

/**
 * Set a new set point for the PID to track.
 * @return The set point
 */
float pidGetDesired(PidObject* pid);

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool pidIsActive(PidObject* pid);

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void pidSetError(PidObject* pid, const float error);

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void pidSetKp(PidObject* pid, const float kp);

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void pidSetKi(PidObject* pid, const float ki);

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void pidSetKd(PidObject* pid, const float kd);

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void pidSetDt(PidObject* pid, const float dt);

void pidSetPID(PidObject* pid, float kp, float ki, float kd);
float constrain(float value, const float minVal, const float maxVal);
#endif /* PID_H_ */

