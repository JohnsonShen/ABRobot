/*============================================================================*
  ___     ___     ___            _                _                           *
  /   \   | _ )   | _ \    ___   | |__     ___    | |_      o O O             *
  | - |   | _ \   |   /   / _ \  | '_ \   / _ \   |  _|    o                  *
  |_|_|   |___/   |_|_\   \___/  |_.__/   \___/   _\__|   TS__[O]             *
_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""| {======|             *
"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'./o--000'             *
 *                                                                            *
 * Auto Balance Robot controller firmware for Nuvoton Cortex M4 series        *
 *                                                                            *
 * Written by T.L. Shen for Nuvoton Technology.                               *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                  *
 *                                                                            *
 *============================================================================*
 */
#include <stdio.h>
#include <math.h>
#include "Def.h"
#if STACK_BARO
#include "pid.h"
#include "stabilizer.h"
#include "RC.h"
#include "sensors.h"
#include "AltHold.h"
#ifdef BMP085
#include "AdaFruit_BMP085.h"
#endif
#ifdef MS5611
#include "ms5611.h"
#endif
#ifdef BMP280
#include "BMP280.h"
#endif
#define NEAR_LAND_TH 0.3f
#define AUTO_LAND_SPEED     0.03f //(Meter/Sec)
#define NEAR_LAND_COUNT_TH 80
#define FINAL_LAND_FACTOR 1.0007f
#define LAND_STATE0 0
#define LAND_STATE1 1
#define LAND_STATE2 2
#define LAND_STATE3 3
// Baro variables
static float asl;     
static float aslRaw;  
static float aslLong; 

// Altitude hold variables
PidObject *altHoldPID; 
bool altHold = false;          
bool setAltHold = false;      
bool nearLand = false;


static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float altHoldPIDVal;                    
static float altHoldErr;                       
static float accFuse;

// Altitude hold & Baro Params
static float altHoldChange          = 0;     
static float altHoldTarget          = -1;    
static float altHoldErrMax          = 5.0;  
static float altHoldChange_SENS     = 30000;   
static float pidAslFac              = 1;
static float pidAlpha               = 0.5;  
//static float vSpeedASLFac           = 1.0;    
//static float vSpeedFac              = 0.2;
static float vSpeedASLDeadband      = 0.005; 
//static float vSpeedASLDeadband      = 0.02; 
//static float vSpeedLimit            = 0.05;  
static float errDeadband            = 0.00;  
//static float vBiasAlpha             = 1.0; 
static float aslAlpha               = 0.91; 
static float aslAlphaLong           = 0.93; 
static uint16_t altHoldMinThrust    = 0; 
static uint16_t altHoldBaseThrust   = (RC_THR_HOV - RC_THR_MIN); 
static uint16_t altHoldMaxThrust    = 350; 

static bool altHoldMode = false;
static bool altHoldModeOld = false;
static bool AutoLandMode = false;
uint16_t NearLandCount = 0;
uint16_t calibratingB = 0;
ALTHOLD_STATE_T AltHoldState;
float finalLandFactor=FINAL_LAND_FACTOR;

void SetCalibratingB(uint8_t c)
{
	calibratingB = c;
}
float getAslSpeed()
{
	return vSpeedASL;
}
float getAccFuse()
{
	return accFuse;
}
float getAltHoldPIDVal()
{
	return altHoldPIDVal;
}
bool GetAltHoldMode()
{
	return altHoldMode;
}
bool GetAutoLandMode()
{
	return AutoLandMode;
}
ALTHOLD_STATE_T* GetAltHoldState()
{
	return &AltHoldState;
}
void InitAltHoldPID()
{
	// Set to current altitude
		altHoldTarget = asl;
		pidReset(altHoldPID);
		pidSetDesired(altHoldPID, asl);

		altHoldPID->integ = 0;
		accFuse = 0;

		// Reset altHoldPID
		altHoldPIDVal = pidUpdate(altHoldPID, asl, false);
}
void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange)
{
	int16_t rcData[RC_CHANS], rc_altitude, rc_alt_hold;
	bool rc_min_throttle;
	
	getRC(rcData);
	if(rcData[THR_CH]<=RC_THR_MIN)
		rc_min_throttle = true;
	else
		rc_min_throttle = false;
	if(checkArm()) {
		rc_altitude = GetRCAltitude();
		if((rc_altitude<RC_ALT_DEAD_BAND)&&(rc_altitude>-RC_ALT_DEAD_BAND)) {
		rc_altitude = 0;
			if(AutoLandMode==true)
				altHoldTarget = asl;
			AutoLandMode = false;
		}
	}
	else {
		rc_altitude = 0;
		AutoLandMode = false;
	}
	
	
	rc_alt_hold = (rcData[AUX1_CH] - RC_ALT_MID);
	if((rc_alt_hold>RC_ALT_DEAD_BAND)||!IsRCConnected())
		altHoldMode = true;
	else if(rc_alt_hold<-RC_ALT_DEAD_BAND) 
		altHoldMode = false;
		
	
	*altHold = altHoldMode; 
	*setAltHold = !altHoldModeOld && altHoldMode; 
	if(*altHold&&!(*setAltHold)&&checkArm()&&(rc_min_throttle||!IsRCConnected())&&IsMotorSpin()) {
		if(AutoLandMode==false) {
			AutoLandMode = true; 
			InitAltHoldPID();
			nearLand = false;
			NearLandCount = 0;
		}
	}
	else if(!*altHold)
		AutoLandMode = false;
	
	*altHoldChange = altHoldMode ? (float) rc_altitude : 0.0f; 
#ifdef DGB
	printf("altHoldPIDVal:%f,altHoldMode:%d, altHoldChange:%f, altHoldTarget:%f, asl:%f\n",altHoldPIDVal,altHoldMode, *altHoldChange,altHoldTarget,asl);
#endif
	AltHoldState.altHoldPIDVal = altHoldPIDVal;
	AltHoldState.altHoldChange = *altHoldChange;
	AltHoldState.altHoldTarget = altHoldTarget;
	AltHoldState.asl = asl;
	altHoldModeOld = altHoldMode;
}
void SetAltHoldPIDObj(PidObject* PIDObj)
{
	altHoldPID = PIDObj;
}
bool GetNearLand()
{
	return nearLand;
}
bool CheckNearLand()
{
	if(((asl - altHoldTarget)>NEAR_LAND_TH)||(NearLandCount>NEAR_LAND_COUNT_TH)){
		nearLand = true;
		return true;
	}
	else if(vSpeedASL==0) {
		NearLandCount++;
		return false;
	}
	else {
		NearLandCount = 0;
		return false;
	}
}

void stabilizerAltHoldUpdate(int16_t *actuatorThrust)
{
	float vSpeed;	

	static float baroGroundTemperatureScale=0,logBaroGroundPressureSum=0,vSpeedLong=0,altholdThrust,finalLandThrust;

	int32_t BaroAlt;
	BaroInfo_T *BaroInfo;
	BaroInfo = GetBaroInfo();
	commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);
  
	if(calibratingB > 0) {
		logBaroGroundPressureSum = log(BaroInfo->baroPressureSum);
#ifdef BMP085
		baroGroundTemperatureScale = (readTemperature(BaroInfo->baroTemperature)*100 + 27315) *  29.271267f;
#else
		baroGroundTemperatureScale = (BaroInfo->baroTemperature*100 + 27315) *  29.271267f;
#endif
		calibratingB--;
	}

	BaroAlt = ( logBaroGroundPressureSum - log(BaroInfo->baroPressureSum) ) * baroGroundTemperatureScale;

	aslRaw = (float)BaroAlt/100;

	asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
	SetBaroAltitude(asl);

	aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

	vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband)*ALTHOLD_UPDATE_FREQ;
//vSpeedASL = vSpeedASL*0.95f + deadband(asl - aslLong, vSpeedASLDeadband)/ALTHOLD_UPDATE_DT*0.05f;

	GetvSpeed(&vSpeed);
	
	vSpeedLong = vSpeedLong*0.95f + vSpeed;

	vSpeedAcc = vSpeed;//vSpeedLong * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);

	// Reset Integral gain of PID controller if being charged
	if (!checkArm()) {
		altHoldPID->integ = 0.0;
	}

	if (setAltHold) {
		InitAltHoldPID();
		altHoldBaseThrust   = GetactuatorThrust();
		altHoldMaxThrust   = altHoldBaseThrust + 70;
		altHoldMinThrust   = altHoldBaseThrust - 70;
	}

	// In altitude hold mode
	if (altHold&&checkArm()) {
		if(AutoLandMode)
			altHoldTarget -= AUTO_LAND_SPEED/ALTHOLD_UPDATE_FREQ;
		else
		altHoldTarget += altHoldChange / altHoldChange_SENS;
		pidSetDesired(altHoldPID, altHoldTarget);

		altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
				-altHoldErrMax, altHoldErrMax);
		pidSetError(altHoldPID, -altHoldErr);

		
		
		/*if(vSpeedASL>0)
			accFuse = -accFuse*(1-vSpeedFac) + sqrt((vSpeedAcc+0.05f)*(vSpeedAcc+0.05f)*(vSpeedASL+0.1f)*(vSpeedASL+0.1f)*vSpeedFac);
		else
			accFuse = accFuse*(1-vSpeedFac) + sqrt((vSpeedAcc-0.05f)*(vSpeedAcc-0.05f)*(vSpeedASL-0.1f)*(vSpeedASL-0.1f)*vSpeedFac);*/
		accFuse = vSpeedASL*0.5f + vSpeedAcc*0.5f;
		
		if(altHoldChange!=0)
			accFuse = 0;
		
		altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.0f - pidAlpha) * (pidUpdate(altHoldPID, asl, false)- accFuse*0.0f) ;
		//printf("%f - %f  %f\n",altHoldPIDVal, pidUpdate(altHoldPID, asl, false),accFuse*20);
		// compute new thrust
		altholdThrust = max(altHoldMinThrust, min(altHoldMaxThrust,
				limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));
		
		if(AutoLandMode) {
			if(nearLand) {
				//finalLandThrust-=0.1f;
				finalLandFactor=FINAL_LAND_FACTOR*finalLandFactor;
				finalLandThrust-=finalLandFactor/10;
				*actuatorThrust = finalLandThrust;
				if(finalLandThrust<=0) {
					AutoLandMode = false;
					MotorDisArm();
					nearLand = false;
				}
			}
			else if(CheckNearLand()) {
				finalLandThrust = altholdThrust;
				finalLandFactor=FINAL_LAND_FACTOR;
			}
			else {
				*actuatorThrust = altholdThrust;
			}
		}
		else
			*actuatorThrust = altholdThrust;
	}
	else {
		altHoldTarget = 0.0;
		altHoldErr = 0.0;
		altHoldPIDVal = 0.0;
		if (checkArm())
			*actuatorThrust = RC_THR_ARM - RC_THR_MIN;
		else
			*actuatorThrust = 0;
	}
#if 0
	if(vSpeedASL>0)
		altHoldPIDVal = altHoldPIDVal*0.8f + sqrt(vSpeedAcc*vSpeedAcc*vSpeedASL)*0.2f;
	else
		altHoldPIDVal = altHoldPIDVal*0.8f - sqrt(vSpeedAcc*vSpeedAcc*-vSpeedASL)*0.2f;
#endif
}
#endif
