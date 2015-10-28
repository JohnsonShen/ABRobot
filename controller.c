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
#include <stdio.h>
#include <stdlib.h>
#include "retarget.h"
#include "controller.h"
#include "pid.h"
#include "param.h"
#include "FlashCtrl.h"
#include "Timer_Ctrl.h"
//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidAltHold; // Used for altitute hold mode. I gets reset when the bat status changes

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;

static bool isInit;

bool LoadFlashPID()
{
	float PID_FIELD[PID_FIELD_SIZE],IMU_UPDATE_DT;
	bool FlashValid;
	FlashValid = GetFlashPID(PID_FIELD);
	IMU_UPDATE_DT = getUpdateDT();
	if(FlashValid) {
		pidInit(&pidRoll, 0, PID_FIELD[0], PID_FIELD[1], PID_FIELD[2], IMU_UPDATE_DT);
		pidInit(&pidPitch, 0, PID_FIELD[3], PID_FIELD[4], PID_FIELD[5], IMU_UPDATE_DT);
		pidInit(&pidYaw, 0, PID_FIELD[6], PID_FIELD[7], PID_FIELD[8], IMU_UPDATE_DT);
		pidInit(&pidRollRate, 0, PID_FIELD[9], PID_FIELD[10], PID_FIELD[11], IMU_UPDATE_DT);
		pidInit(&pidPitchRate, 0, PID_FIELD[12], PID_FIELD[13], PID_FIELD[14], IMU_UPDATE_DT);
		pidInit(&pidYawRate, 0, PID_FIELD[15], PID_FIELD[16], PID_FIELD[17], IMU_UPDATE_DT);
		pidInit(&pidAltHold, 0, PID_FIELD[18], PID_FIELD[19], PID_FIELD[20], ALTHOLD_UPDATE_DT);
		return true;
	}
	else {
		pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
		pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
		pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
		pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
		pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
		pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);
		pidInit(&pidAltHold, 0, PID_ALTHOLD_KP, PID_ALTHOLD_KI, PID_ALTHOLD_KD, ALTHOLD_UPDATE_DT);
		return false;
	}
}
void controllerInit()
{
	if(isInit)
		return;

	if(LoadFlashPID()==false)
		printf("Load PID from [DEFAULT]\n");
	else
		printf("Load PID from [FLASH]\n");

	pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

	pidSetIntegralLimitLow(&pidRollRate, -PID_ROLL_RATE_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidPitchRate, -PID_PITCH_RATE_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidYawRate, -PID_YAW_RATE_INTEGRATION_LIMIT);

	pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
	pidSetIntegralLimitLow(&pidRoll, -PID_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidPitch, -PID_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimitLow(&pidYaw, -PID_YAW_INTEGRATION_LIMIT);

	isInit = true;
}

bool controllerTest()
{
	return isInit;
}

void controllerCorrectRatePID(
				float rollRateActual, float pitchRateActual, float yawRateActual,
				float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
	float pidOut;
	pidSetDesired(&pidRollRate, rollRateDesired);
	pidSetDt(&pidRollRate,getUpdateDT());
	pidOut = pidUpdate(&pidRollRate, rollRateActual, TRUE);
	TRUNCATE_SINT16(rollOutput, pidOut);
	//printf("%f %f %f %f\n",pidOut,pidRollRate.outP,pidRollRate.outI,pidRollRate.outD);
	pidSetDesired(&pidPitchRate, pitchRateDesired);
	pidSetDt(&pidPitchRate,getUpdateDT());
	pidOut = pidUpdate(&pidPitchRate, pitchRateActual, TRUE);
	TRUNCATE_SINT16(pitchOutput, pidOut);

	//printf("%f %f %f %f\n",pidOut,pidPitchRate.outP,pidPitchRate.outI,pidPitchRate.outD);
	pidSetDesired(&pidYawRate, yawRateDesired);
	pidSetDt(&pidYawRate,getUpdateDT());
	pidOut = pidUpdate(&pidYawRate, yawRateActual, TRUE);
	TRUNCATE_SINT16(yawOutput, pidOut);
}

void controllerCorrectAttitudePID(
				float eulerRollActual, float eulerPitchActual, float eulerYawActual,
				float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
				float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
	float yawError;
	pidSetDesired(&pidRoll, eulerRollDesired);
	pidSetDt(&pidRoll,getUpdateDT());
	*rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, TRUE);

	// Update PID for pitch axis
	pidSetDesired(&pidPitch, eulerPitchDesired);
	pidSetDt(&pidPitch,getUpdateDT());
	*pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, TRUE);

	// Update PID for yaw axis
	yawError = eulerYawDesired - eulerYawActual;
	if (yawError > 180.0f)
		yawError -= 360.0f;
	else if (yawError < -180.0f)
		yawError += 360.0f;
	
	pidSetError(&pidYaw, yawError);
	pidSetDt(&pidYaw,getUpdateDT());
	*yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, FALSE);
}

void controllerResetAllPID(void)
{
	pidReset(&pidRoll);
	pidReset(&pidPitch);
	pidReset(&pidYaw);
	pidReset(&pidRollRate);
	pidReset(&pidPitchRate);
	pidReset(&pidYawRate);
}

void controllerSetRollPID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidRoll, kp, pidRoll.ki, pidRoll.kd);
	if(ki>=0)
		pidSetPID(&pidRoll, pidRoll.kp, ki, pidRoll.kd);
	if(kd>=0)
		pidSetPID(&pidRoll, pidRoll.kp, pidRoll.ki, kd);
}

void controllerSetPitchPID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidPitch, kp, pidPitch.ki, pidPitch.kd);
	if(ki>=0)
		pidSetPID(&pidPitch, pidPitch.kp, ki, pidPitch.kd);
	if(kd>=0)
		pidSetPID(&pidPitch, pidPitch.kp, pidPitch.ki, kd);
}
void controllerSetYawPID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidYaw, kp, pidYaw.ki, pidYaw.kd);
	if(ki>=0)
		pidSetPID(&pidYaw, pidYaw.kp, ki, pidYaw.kd);
	if(kd>=0)
		pidSetPID(&pidYaw, pidYaw.kp, pidYaw.ki, kd);
}
void controllerSetRollRatePID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidRollRate, kp, pidRollRate.ki, pidRollRate.kd);
	if(ki>=0)
		pidSetPID(&pidRollRate, pidRollRate.kp, ki, pidRollRate.kd);
	if(kd>=0)
		pidSetPID(&pidRollRate, pidRollRate.kp, pidRollRate.ki, kd);
}
void controllerSetPitchRatePID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidPitchRate, kp, pidPitchRate.ki, pidPitchRate.kd);
	if(ki>=0)
		pidSetPID(&pidPitchRate, pidPitchRate.kp, ki, pidPitchRate.kd);
	if(kd>=0)
		pidSetPID(&pidPitchRate, pidPitchRate.kp, pidPitchRate.ki, kd);
}
void controllerSetYawRatePID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidYawRate, kp, pidYawRate.ki, pidYawRate.kd);
	if(ki>=0)
		pidSetPID(&pidYawRate, pidYawRate.kp, ki, pidYawRate.kd);
	if(kd>=0)
		pidSetPID(&pidYawRate, pidYawRate.kp, pidYawRate.ki, kd);
}
void controllerSetAltHoldPID(float kp, float ki, float kd)
{
	if(kp>=0)
		pidSetPID(&pidAltHold, kp, pidAltHold.ki, pidAltHold.kd);
	if(ki>=0)
		pidSetPID(&pidAltHold, pidAltHold.kp, ki, pidAltHold.kd);
	if(kd>=0)
		pidSetPID(&pidAltHold, pidAltHold.kp, pidAltHold.ki, kd);
}
void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
	*roll = rollOutput;
	*pitch = pitchOutput;
	*yaw = yawOutput;
}
void GetRollPID(float* PID)
{
	PID[0] = pidRoll.kp;
	PID[1] = pidRoll.ki;
	PID[2] = pidRoll.kd;
}
void GetPitchPID(float* PID)
{
	PID[0] = pidPitch.kp;
	PID[1] = pidPitch.ki;
	PID[2] = pidPitch.kd;
}
void GetYawPID(float* PID)
{
	PID[0] = pidYaw.kp;
	PID[1] = pidYaw.ki;
	PID[2] = pidYaw.kd;
}
void GetRollRatePID(float* PID)
{
	PID[0] = pidRollRate.kp;
	PID[1] = pidRollRate.ki;
	PID[2] = pidRollRate.kd;
}
void GetPitchRatePID(float* PID)
{
	PID[0] = pidPitchRate.kp;
	PID[1] = pidPitchRate.ki;
	PID[2] = pidPitchRate.kd;
}
void GetYawRatePID(float* PID)
{
	PID[0] = pidYawRate.kp;
	PID[1] = pidYawRate.ki;
	PID[2] = pidYawRate.kd;
}
void GetAltHoldPID(float* PID)
{
	PID[0] = pidAltHold.kp;
	PID[1] = pidAltHold.ki;
	PID[2] = pidAltHold.kd;
}
int GetPIDValue()
{
	char value_s[3];
	
	value_s[0] = GetChar();
	value_s[1] = GetChar();
	value_s[2] = GetChar();
	
	return atoi(value_s);
}
float GetPIDfloat()
{
	char value_s[4];
	
	value_s[0] = GetChar();
	value_s[1] = GetChar();
	value_s[2] = GetChar();
	value_s[3] = GetChar();
	
	return atof(value_s);
}
PidObject* GetAltHoldPIDObj()
{
	return &pidAltHold;
}
void SetPID()
{
	char type = GetChar();
	//int value;
	char rpy, pid;
	float valuef;
	if(type=='p') { //PID
		rpy = GetChar();
		if(rpy=='r') { //Roll
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p')
				controllerSetRollPID(valuef, -1, -1);
			else if(pid=='i') 
				controllerSetRollPID(-1, valuef, -1);
			else if(pid=='d') 
				controllerSetRollPID(-1, -1, valuef);
		}
		else if(rpy=='p') { //Pitch
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p') 
				controllerSetPitchPID(valuef, -1, -1);
			else if(pid=='i') 
				controllerSetPitchPID(-1, valuef, -1);
			else if(pid=='d') 
				controllerSetPitchPID(-1, -1, valuef);
		}
		else if(rpy=='y') { //Yaw
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p') 
				controllerSetYawPID(valuef, -1, -1);
			else if(pid=='i') 
				controllerSetYawPID(-1, valuef, -1);
			else if(pid=='d') 
				controllerSetYawPID(-1, -1, valuef);
		}
	}
	else if(type=='r') { //Rate PID
		rpy = GetChar();
		if(rpy=='r') { //Roll
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p') 
				controllerSetRollRatePID((float)valuef, -1, -1);
			else if(pid=='i') 
				controllerSetRollRatePID(-1, (float)valuef, -1);
			else if(pid=='d') 
				controllerSetRollRatePID(-1, -1, (float)valuef);
		}
		else if(rpy=='p') { //Pitch
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p') 
				controllerSetPitchRatePID((float)valuef, -1, -1);
			else if(pid=='i') 
				controllerSetPitchRatePID(-1, (float)valuef, -1);
			else if(pid=='d') 
				controllerSetPitchRatePID(-1, -1, (float)valuef);
		}
		else if(rpy=='y') { //Yaw
			pid = GetChar();
			valuef = GetPIDfloat();
			if(pid=='p') 
				controllerSetYawRatePID((float)valuef, -1, -1);
			else if(pid=='i') 
				controllerSetYawRatePID(-1, (float)valuef, -1);
			else if(pid=='d') 
				controllerSetYawRatePID(-1, -1, (float)valuef);
		}
	}
	else if(type=='a') { //Altitide Hold PID
		pid = GetChar();
		valuef = GetPIDfloat();
		if(pid=='p') 
			controllerSetAltHoldPID((float)valuef, -1, -1);
		else if(pid=='i') 
			controllerSetAltHoldPID(-1, (float)valuef, -1);
		else if(pid=='d') 
			controllerSetAltHoldPID(-1, -1, (float)valuef);
	}
	else if(type=='s') { //'s'tore flash PID
		UpdateFlashPID(false);
	}
	else if(type=='l') { //'l'oad flash PID
		LoadFlashPID();
	}
}

