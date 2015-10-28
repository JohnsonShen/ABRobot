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
#include <stdlib.h>
#include "retarget.h"
#include "AHRSLib.h"
#include "Timer_Ctrl.h"
#include "FlashCtrl.h"
#include "Sensors.h"
#include "Report.h"
#include "LED.h"
#include "RC.h"
#define MAG_CAL_SUCESS_TH 20
uint8_t CalQFactor;
void CalibrationFail()
{
	const char fail = 'f';
	Serial_write((char*)&fail, 1);
}
void AccCalibration()
{
	const char direction = GetChar();
	const char calibration_done = 'd';
	char side = atoi(&direction);
	signed char status;
	
	if (direction == 'x')
		side = 0;
	else
		side = atoi(&direction);
	
	if ((direction == '0')||(direction == 'x'))
		nvtCalACCInit();
	
	do {
		DelayMsec(1);
		SensorsRead(SENSOR_ACC,1);
		status = nvtCalACCBufferFill(side);
	}while(status==STATUS_BUFFER_NOT_FILLED);
	
	if(status==STATUS_BUFFER_FILLED) {
		if (direction == 'x')
			UpdateFlashCal(SENSOR_ACC, false);
		Serial_write((char*)&direction, 1);
	}
	else {
	Serial_write((char*)&calibration_done, 1);
		UpdateFlashCal(SENSOR_ACC, false);
	}
}
void GyroCalibration()
{
	const char axis_done = GetChar();
	const char axis = axis_done - 0x78;
	const char calibration_done = 'd';
	
	signed char status;
	
	nvtCalGyroInit(axis);
	
	do {
		SensorsRead(SENSOR_GYRO,1);
#ifndef OPTION_RC
		DelayMsec(5);
#else
		DelayMsec(16);
#endif
		//printf("T:%d\n",getTickCount());
		status=nvtGyroScaleCalibrate(axis);	
		led_arm_state(LED_STATE_TOGGLE);
		UpdateLED();
	} while(status==STATUS_GYRO_CAL_RUNNING);	
	
	if(status==STATUS_GYRO_AXIS_CAL_DONE)
		Serial_write((char*)&axis_done, 1);
	else {
		Serial_write((char*)&calibration_done, 1);
		UpdateFlashCal(SENSOR_GYRO, false);
	}
}
void MagCalibration()
{
	char calibration_done;
	signed char status;
	int16_t RawMAG[3];
	
	nvtCalMAGInit();
	do {
#ifndef OPTION_RC
		DelayMsec(160);
#else
		DelayMsec(320);
#endif
		SensorsRead(SENSOR_MAG,1);
		status = nvtCalMAGBufferFill();
		nvtGetSensorRawMAG(RawMAG);
		if (report_format == REPORT_FORMAT_BINARY) {
			Serial_write((char*)RawMAG, 6);
		}
		else if (report_format == REPORT_FORMAT_TEXT) {
			printf("@rM:%d,%d,%d\n",RawMAG[0],RawMAG[1],RawMAG[2]);
		}
	}while(status==STATUS_BUFFER_NOT_FILLED);
	
	if(status==STATUS_CAL_DONE) {
		CalQFactor = nvtGetMagCalQFactor();
		if(CalQFactor<MAG_CAL_SUCESS_TH)
			calibration_done = 'd';
		else
			calibration_done = 'f';
		if (report_format == REPORT_FORMAT_BINARY) {
			Serial_write((char*)&calibration_done, 1);
			Serial_write((char*)&CalQFactor, 1);
		}
		else if (report_format == REPORT_FORMAT_TEXT) {
			printf("%c,%d\n",calibration_done,CalQFactor);
		}
	}
}
void SensorCalibration()
{
	char InitState = GetSensorInitState();
	char calibration_sensor = GetChar();
#ifdef OPTION_RC	
	TIMER_Enable(false); 
	RC_Enable(false);
#endif
	if((calibration_sensor=='a')&&(InitState&SENSOR_ACC)) {       // Do 'a'cc calibration
		AccCalibration();
	}
	else if((calibration_sensor=='g')&&(InitState&SENSOR_GYRO)) {// Do 'g'yro calibration 
		GyroCalibration();
	}
	else if((calibration_sensor=='m')&&(InitState&SENSOR_MAG)) { // Do 'g'yro calibration 
		MagCalibration();
		if(CalQFactor<MAG_CAL_SUCESS_TH)
		UpdateFlashCal(SENSOR_MAG, false);
		SensorInitMAG();
	}
	else                                                        // Fail doing calibration 
		CalibrationFail();
#ifdef OPTION_RC	
	TIMER_Enable(true);
	RC_Enable(true);
#endif
}
