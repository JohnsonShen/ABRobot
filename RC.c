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
#ifdef M451
#include "M451Series.h"
#else
#include "DrvGPIO.h"
#endif
#include "Def.h"
#include "retarget.h"
#include "Timer_Ctrl.h"
#include "RC.h"
#include "LED.h"
#include "RC_pwm.h"
#include "RC_ssv.h"
#include "stabilizer.h"
#include "AltHold.h"

int16_t rcData[RC_CHANS];
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502};

static char motorArm = 0;
static char Calbrate = 0;
static int arm_count = 0;
static int idle_count = 0;
bool beSSVConnected = false;

void RC_Enable(char enable)
{
	if(IsSSVConnected())
	RC_SSV_Enable(enable);
	else
		RC_PWM_Enable(enable);
}
void RC_Init(void)
{
	char i;
	for(i = 0; i<RC_CHANS; i++)
		rcData[i] = 1500;
	
	beSSVConnected = RC_SSV_Init();
	if(beSSVConnected)
		printf("SSV Receiver Detected.\n");
	else {
	RC_PWM_Init();
	RC_PWM_Enable(true);
		RC_SSV_Enable(false);
	}
}
uint16_t readRCChannel(uint8_t chan) {
	if(IsSSVConnected())
	return readSsvRC(chan);
	else 
		return readPwmRC(chan);
}
void computePwmRC()
{
	static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan,a;

	rc4ValuesIndex++;
	
	if (rc4ValuesIndex == 4) 
		rc4ValuesIndex = 0;
	
	for (chan = 0; chan < RC_CHANS; chan++) {
		rcData4Values[chan][rc4ValuesIndex] = readRCChannel(chan);
		rcDataMean[chan] = 0;
		
		for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
			rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
		
		if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  
			rcData[chan] = rcDataMean[chan]+2;
			
		if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  
			rcData[chan] = rcDataMean[chan]-2;
		
	}
}
void computeSSVRC()
{
	uint8_t chan;
	
	for (chan = 0; chan < RC_CHANS; chan++) 
		rcData[chan] = readRCChannel(chan);
}	
void computeRC()
{
	if(IsSSVConnected())
		computeSSVRC();
	else
		computePwmRC();
}
void MotorArm()
{
	motorArm = 1;
	arm_count = 0;
	idle_count = 0;
	ClearFlip();
	if(IsSSVConnected())
	RC_CheckFlyMode();
}
bool IsRCConnected()
{
	if(IsSSVConnected())
	return IsSSVRCConnected();
	else
	return IsPWMRCConnected();
}
bool IsSSVConnected()
{
	return beSSVConnected;
}
void MotorDisArm()
{
	motorArm = 0;
	arm_count = 0;
	idle_count = 0;
	Calbrate = 0;
	led_arm_state(LED_STATE_OFF);
}
void armDetect()
{
	if(motorArm==0) {
#if STACK_BARO
		if((rcData[THR_CH]<RC_MIN_CHECK)&&(rcData[YAW_CH]>RC_ARM_MAX_CHECK)&&(!GetAltHoldMode()))
#else
		if((rcData[THR_CH]<RC_MIN_CHECK)&&(rcData[YAW_CH]>RC_ARM_MAX_CHECK))
#endif
		{
			arm_count++;
			if(arm_count*OUTPUT_RC_INTERVAL>ARM_DETECT_LATENCY)
				MotorArm();
		}
		else
			arm_count = 0;
	}
	else {
			if(Calbrate==0) {
				if(nvtGyroCenterCalibrate()!=STATUS_GYRO_CAL_DONE)
					led_arm_state(LED_STATE_TOGGLE);
				else {
					Calbrate = 1;
					led_arm_state(LED_STATE_ON);
				}
		}else if(rcData[THR_CH]<RC_MIN_CHECK) {
			if(!GetAutoLandMode())
				idle_count++;
			if(rcData[YAW_CH]<RC_ARM_MIN_CHECK) {
				arm_count++;
				if(arm_count*OUTPUT_RC_INTERVAL>ARM_DETECT_LATENCY)
				MotorDisArm();
			}
			else {
				if(idle_count*OUTPUT_RC_INTERVAL>(ARM_DETECT_LATENCY*15))
					MotorDisArm();
			}	
		}
		else {
			idle_count = 0;
			arm_count = 0;
		}
		
		if(GetFlip()) 
			MotorDisArm();
	}
}
bool checkArm()
{
	return Calbrate;
}
void getRC(int16_t* rc)
{
	char i;
	for(i = 0; i<RC_CHANS; i++)
		rc[i] = rcData[i];
}
uint16_t* getValue()
{
	return (uint16_t*)rcValue;
}
int16_t GetRCThrust()
{
	if(IsSSVConnected())
		return GetSSVThrust();
	else
		return GetPWMThrust();
}
int16_t GetRCAltitude()
{
	if(IsSSVConnected())
		return GetSSVAltitude();
	else
		return GetPWMAltitude();
}
