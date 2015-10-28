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
#include "AHRSLib.h"
#include "Report.h"
#include "retarget.h"
#include "Sensors.h"
#include "stabilizer.h"
#include "controller.h"
#include "RC.h"
#include "AltHold.h"
#include "battery.h"
#include "ssv7241.h"
#include "RC_ssv.h"
#include "motors.h"
#include "gps.h"
typedef struct {
	int16_t rcData[RC_CHANS];
	float  rssif;
	int8_t arm;
	int8_t mag;
	int8_t headfree;
	int8_t althold;
	int8_t battery;
	int8_t flymode;
	int8_t matching;
	int8_t autolanding;
	int8_t usessv;
	int8_t rcconnected;
}RC_STATE_T;
char report_mode = REPORT_AHRS_EULER;
char report_format = REPORT_FORMAT_TEXT;
char stream_mode = STREAM_PAUSE;

void report_ahrs_euler()
{
	float Euler[3],Altitude;
	
	nvtGetEulerRPY(Euler);
#if STACK_BARO
	Altitude = GetBaroAltitude();
#else
	Altitude = 0;
#endif
	if (report_format == REPORT_FORMAT_BINARY) {
		float rpy[4];  
		rpy[0] = Euler[0];
		rpy[1] = Euler[1];
		rpy[2] = Euler[2];
		rpy[3] = Altitude;
		Serial_write((char*)rpy, 16);
	}
	else if (report_format == REPORT_FORMAT_TEXT)
		printf("@RPYA:%f,%f,%f,%f\n",Euler[0],Euler[1],Euler[2],Altitude);
}
void report_ahrs_quaternion()
{
	float Quaternion[4];
	nvtGetQuaternion(Quaternion);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)&Quaternion[0], 4);
		Serial_write((char*)&Quaternion[1], 4);
		Serial_write((char*)&Quaternion[2], 4);
		Serial_write((char*)&Quaternion[3], 4);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@Quaternion:%f,%f,%f,%f\n",Quaternion[0],Quaternion[1],Quaternion[2],Quaternion[3]);
	}
}
void report_sensor_raw()
{
	int16_t RawACC[3], RawGYRO[3], RawMAG[3];
	uint16_t RawBARO[2];
	
	nvtGetSensorRawACC(RawACC);
	nvtGetSensorRawGYRO(RawGYRO);
	nvtGetSensorRawMAG(RawMAG);
	nvtGetSensorRawBARO(RawBARO);
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)RawACC, 6);
		Serial_write((char*)RawGYRO, 6);
		Serial_write((char*)RawMAG, 6);
		Serial_write((char*)RawBARO, 4);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@rA:%d,%d,%d  ",RawACC[0],RawACC[1],RawACC[2]);
		printf("@rG:%d,%d,%d  ",RawGYRO[0],RawGYRO[1],RawGYRO[2]);
		printf("@rM:%d,%d,%d  ",RawMAG[0],RawMAG[1],RawMAG[2]);
		printf("@rB:%d,%d,\n",RawBARO[0], RawBARO[1]);
	}
}

void report_sensor_calibrated()
{
	float CalACC[3], CalGYRO[3], CalMAG[3], CalBaro;
	nvtGetCalibratedACC(CalACC);
	nvtGetCalibratedGYRO(CalGYRO);
	nvtGetCalibratedMAG(CalMAG);
#if STACK_BARO
	CalBaro = GetBaroAltitude();
#else
	CalBaro = 0;
#endif
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)CalACC, 12);
		Serial_write((char*)CalGYRO, 12);
		Serial_write((char*)CalMAG, 12);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@cA:%f,%f,%f  ",CalACC[0],CalACC[1],CalACC[2]);
		printf("@cG:%f,%f,%f  ",CalGYRO[0],CalGYRO[1],CalGYRO[2]);
		printf("@cM:%f,%f,%f  ",CalMAG[0],CalMAG[1],CalMAG[2]);
		printf("@cB:%f\n",CalBaro);
	}
}
void report_motor_power()
{
	uint16_t MotorPower[MOTOR_NUMBER];
	GetMotorPower(MotorPower);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)MotorPower, 8);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
#ifdef HEX6X
		printf("@motorP:%d,%d,%d,%d,%d,%d \n",MotorPower[0],MotorPower[1],MotorPower[2],MotorPower[3],MotorPower[4],MotorPower[5]);
#else
		printf("@motorP:%d,%d,%d,%d \n",MotorPower[0],MotorPower[1],MotorPower[2],MotorPower[3]);
#endif
	}
}

void report_pid()
{
	float PID[9];
	GetRollPID(&PID[0]);
	GetPitchPID(&PID[3]);
	GetYawPID(&PID[6]);

	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)PID, 36);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@mPID:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		PID[0],PID[1],PID[2],PID[3],PID[4],PID[5],PID[6],PID[7],PID[8]);
	}
}

void report_rate_pid()
{
	float PID[9];
	GetRollRatePID(&PID[0]);
	GetPitchRatePID(&PID[3]);
	GetYawRatePID(&PID[6]);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)PID, 36);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@mRatePID:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		PID[0],PID[1],PID[2],PID[3],PID[4],PID[5],PID[6],PID[7],PID[8]);
	}
}
void report_althold_pid()
{
	float PID[3];
	GetAltHoldPID(&PID[0]);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)PID, 12);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@mAltHoldPID:%f,%f,%f\n",
		PID[0],PID[1],PID[2]);
	}
}
void report_velocity()
{
	float Ve[3],Move[3];
	nvtGetVelocity(Ve);
	nvtGetMove(Move);
	nvtGetAccZWithoutGravity(&Ve[0], &Ve[1]);
#if STACK_BARO
	//Ve[0] = getAltHoldPIDVal();
	Ve[0] = getAccFuse();
	Ve[1] = getAslSpeed();
#endif
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)Ve, 12);
		Serial_write((char*)Move, 12);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@mVe_Move:%f,%f,%f,%f,%f,%f\n", Ve[0], Ve[1], Ve[2],Move[0], Move[1], Move[2]);
	}
}
void report_rc_status()
{
	RC_STATE_T RcState;

	RcState.arm = (int8_t)checkArm();
	RcState.mag = (int8_t)getMagMode();
	RcState.headfree = (int8_t)getHeadFreeMode();
#if STACK_BARO
	RcState.althold = (int8_t)GetAltHoldMode();
#else
	RcState.althold = 0;
#endif
	RcState.battery = (int8_t)GetBattery();
	RcState.flymode = RC_GetFlyMode();
	RcState.matching = (int8_t)GetMatchAddressProcess();
#if STACK_BARO
	RcState.autolanding = (int8_t)GetAutoLandMode();
#else
	RcState.autolanding = 0;
#endif
	RcState.usessv = IsSSVConnected();
	RcState.rssif = (float)((int)(GetRSSIf()*1000))/1000;
	RcState.rcconnected = IsRCConnected();
	
	getRC(&RcState.rcData[0]);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)&RcState.rcData[0], 26);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@RC6,A,B,C:%d,%d,%d,%d,%d,%d - %d,%d,%d,%f   \n",RcState.rcData[0],RcState.rcData[1],RcState.rcData[2],RcState.rcData[3],RcState.rcData[4],RcState.rcData[5],
		RcState.arm, RcState.althold,RcState.rcconnected,RcState.rssif);
	}
}
void report_mode_status()
{
	int8_t mode[9];
	float RSSIf;

	mode[0] = (int8_t)checkArm();
	mode[1] = (int8_t)getMagMode();
	mode[2] = (int8_t)getHeadFreeMode();
#if STACK_BARO
	mode[3] = (int8_t)GetAltHoldMode();
#endif
	mode[4] = (int8_t)GetBattery();
	mode[5] = (int8_t)GetRSSI();
	mode[6] = RC_GetFlyMode();
	mode[7] = (int8_t)GetMatchAddressProcess();
#if STACK_BARO
	mode[8] = (int8_t)GetAutoLandMode();
#else
	mode[8] = 0;
#endif
	RSSIf = GetRSSIf();
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)mode, 9);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@Arm,Mag,HFree,Alt,Bat,RSSI,FlyMode,Match,Land:%d,%d,%d,%d,%d(%d),%f,%d,%d,%d\n",mode[0],mode[1],mode[2],mode[3],mode[4],CheckLowBattery(),RSSIf,mode[6],mode[7],mode[8]);
	}
}
void report_flash_status()
{
	CAL_FLASH_STATE_T *FlashState;
	uint8_t State[4];
	FlashState = GetFlashState();
	State[0] = (uint8_t)FlashState->ACC_FLASH;
	State[1] = (uint8_t)FlashState->GYRO_FLASH;
	State[2] = (uint8_t)FlashState->MAG_FLASH;
	State[3] = (uint8_t)FlashState->MAG_QFACTOR;
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)State, 4);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@On Flash:ACC,GYRO,MAG,QFactor:%d,%d,%d,%d\n",State[0],State[1],State[2],State[3]);
	}
}
#ifdef GPS
void report_gps_status()
{
  GPS_Info_T* GPSinfo;
  GPS_Report_T GPSreport;
  GPSinfo = GetGPSInfo();
  GPSreport.GPS_coord[0] = GPSinfo->GPS_coord[0];
  GPSreport.GPS_coord[1] = GPSinfo->GPS_coord[1];
  GPSreport.GPS_numSat = GPSinfo->GPS_numSat;
  GPSreport.GPS_Fixed = GPSinfo->GPS_Fixed;
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)&GPSreport, 10);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@GPS:LAT,LON,Num,Fix:%d,%d,%d,%d\n",GPSreport.GPS_coord[0],GPSreport.GPS_coord[1],GPSreport.GPS_numSat,GPSreport.GPS_Fixed);
	}
}
#endif
#if STACK_BARO
void report_althold_status()
{
	ALTHOLD_STATE_T *AltState = GetAltHoldState();
	uint16_t actuatorThrust = GetactuatorThrust();
	uint16_t altHold = (uint16_t)GetAltHoldMode();
	float vSpeed;
	GetvSpeed(&vSpeed);
	
	if (report_format == REPORT_FORMAT_BINARY) {
		Serial_write((char*)&altHold, 2);
		Serial_write((char*)&actuatorThrust, 2);
		Serial_write((char*)AltState, 16);
	}
	else if (report_format == REPORT_FORMAT_TEXT) {
		printf("@Mod,Thr,PID,Chg,Tgt,Asl,AslS,Vs,Diff,Landing,NearL:%d,%d,%f,%f,%f,%f,%f,%f,%d\n"
			,altHold
			,actuatorThrust
			,AltState->altHoldPIDVal
			,AltState->altHoldChange
			,AltState->altHoldTarget
			,AltState->asl
			,getAslSpeed()
			,vSpeed
			,GetAutoLandMode()
		);
	}
}
#endif
void CheckVersion()
{
	// Read ID
	char mode = GetChar();
	unsigned char Version[2];
	
	if (mode == 'f') {// 'f'irmware version 
		Version[0] = '@';
		Version[1] = VERSION_CODE;
		if (report_format == REPORT_FORMAT_BINARY)
			Serial_write((char*)&Version, 2);
		else
			printf("Firmware Version:%d\n",Version[1]);
	}
	else if (mode == 'b') {// 'b'oard version
		Version[0] = '$';
		Version[1] = BOARD_CODE;
		if (report_format == REPORT_FORMAT_BINARY)
			Serial_write((char*)&Version, 2);
		else
			printf("Board Version:%d\n",Version[1]);
	}
	
}
void report_status()
{
	char mode = GetChar();
	if (mode == 'r') // 'r'c input status
		report_mode = REPORT_RC_STATUS;	
	else if (mode == 'a') // 'a'ltitude hold status
		report_mode = REPORT_ALTHOLD_STATUS;  
	else if (mode == 'm') // 'm'ode status
		report_mode = REPORT_MODE_STATUS;  
	else if (mode == 'f') // 'm'ode status
		report_mode = REPORT_FLASH_STATUS; 
  else if (mode == 'g') // 'g'ps status
		report_mode = REPORT_GPS_STATUS; 
}
void report_sensors()
{
	if(stream_mode==STREAM_PAUSE)
		return;
	
	if (report_mode == REPORT_AHRS_EULER) {
		report_ahrs_euler();
	}
	else if (report_mode == REPORT_AHRS_QUATERNION) {
		report_ahrs_quaternion();
	}
	else if (report_mode == REPORT_SENSORS_RAW) {
		report_sensor_raw();
	}
	else if (report_mode == REPORT_SENSORS_CALIBRATED) {
		report_sensor_calibrated();
	}
	else if (report_mode == REPORT_MOTOR_POWER) {
		report_motor_power();
	}
	else if (report_mode == REPORT_PID) {
		report_pid();
	}
	else if (report_mode == REPORT_RATE_PID) {
		report_rate_pid();
	}
	else if (report_mode == REPORT_ALTHOLD_PID) {
		report_althold_pid();
	}
	else if (report_mode == REPORT_VELOCITY) {
		report_velocity();
	}
	else if (report_mode == REPORT_RC_STATUS) {
		report_rc_status();
	}
	else if (report_mode == REPORT_ALTHOLD_STATUS) {
#if STACK_BARO
		report_althold_status();
#endif
	}
	else if (report_mode == REPORT_MODE_STATUS) {
		report_mode_status();
	}
	else if (report_mode == REPORT_FLASH_STATUS) {
		report_flash_status();
	}
#ifdef GPS
  else if (report_mode == REPORT_GPS_STATUS) {
		report_gps_status();
  }
#endif
}
