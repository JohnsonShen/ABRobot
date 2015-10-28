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
#ifdef M451
#include "M451Series.h"
#else
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvUSB.h"
#include "Driver\DrvI2C.h"
#endif
#include "Def.h"
#include "FlashCtrl.h"
#include "NVT_I2C.h"
#include "Timer_Ctrl.h"
#include "I2CDev.h"
#include "retarget.h"
#include "AHRSLib.h"
#include "Sensors.h"
#include "Report.h"
#include "Calibrate.h"
#include "LED.h"
#include "battery.h"
#include "motors.h"
#include "gps.h"
#ifdef OPTION_RC
#include "RC.h"
#include "RC_ssv.h"
#include "ssv7241.h"
#include "stabilizer.h"
#include "controller.h"
int freq=0;
#endif
#define MAG_INTERVAL 4
void setupSystemClock()
{
#ifdef M451
	SYS_UnlockReg();
	/* Enable HIRC clock */
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

	/* Waiting for HIRC clock ready */
	CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

	/* Switch HCLK clock source to HIRC */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

	/* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
	CLK_SetCoreClock(SYSTEM_CLOCK);
	CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);
	SYS_LockReg();
#else
	uint32_t u32PllCr;
	uint16_t i;
	UNLOCKREG();	
	DrvSYS_SetOscCtrl(E_SYS_OSC22M, 1);
	while (DrvSYS_GetChipClockSourceStatus(E_SYS_OSC22M) != 1);
	DrvSYS_SelectPLLSource(E_SYS_INTERNAL_22M);
	u32PllCr = DrvSYS_GetPLLContent(E_SYS_INTERNAL_22M, SYSTEM_CLOCK);	
	/*Delay for 12M or 22M stable*/
	for (i=0;i<10000;i++);		

	DrvSYS_SetPLLContent(u32PllCr);
	SYSCLK->PLLCON.OE     = 0;
	SYSCLK->PLLCON.PD 	  = 0;

	/*Delay for PLL stable*/
	for (i=0;i<10000;i++);
	/* Change HCLK clock source to be PLL. */
	DrvSYS_SelectHCLKSource(2);
	LOCKREG();	// Lock the protected registers
#endif
}
void setupCommandUART()
{
	/* Enable peripheral clock */
	CLK_EnableModuleClock(UART0_MODULE);
	/* Peripheral clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
	/* Set PD multi-function pins for UART0 RXD, TXD */
	SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
	/* Reset UART module */
	SYS_ResetModule(UART0_RST);
	/* Configure UART0 and set UART0 Baudrate */
	UART_Open(UART0, 115200);
}
void setupUART()
{
#ifdef M451
	setupCommandUART();
#else
	STR_UART_T param;
	DrvGPIO_InitFunction(E_FUNC_UART1);
	DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC,3);
	param.u32BaudRate        = UART_BAUD_RATE;
	param.u8cDataBits        = DRVUART_DATABITS_8;
	param.u8cStopBits        = DRVUART_STOPBITS_1;
	param.u8cParity          = DRVUART_PARITY_NONE;
	param.u8cRxTriggerLevel  = DRVUART_FIFO_1BYTES;
	param.u8TimeOut        	 = 0;
	DrvUART_Open(UART_PORT1, &param);
#endif
	printf("Version:%d\n", VERSION_CODE);
}

void setup()
{
	setupSystemClock();
	setup_system_tick(SYSTEM_TICK_FREQ);
	setupUART();
#ifdef GPS
	setupGPS();
#endif
	I2C_Init();
	FlashInit();
	UpdateBoardVersion(false);
#ifdef OPTION_RC
	RC_Init();
	if(IsSSVConnected())
		Battery_Init();
	LED_Init();
	TIMER_Init();
	stabilizerInit();
#endif
	nvtAHRSInit();
	SensorsInit();
	ChronographSet(ChronMain);
}
void CommandProcess()
{
	// Read incoming control messages
	if (Serial_available() >= 2)
	{
		char start=Serial_read();
		if (start == '@') {// Start of new control message
			int command = Serial_read(); // Commands
			if (command == 'h') {//Hook AHRS Stack Device
				// Read ID
				char id[2];
				id[0] = GetChar();
				id[1] = GetChar();
				// Reply with synch message
				printf("@HOOK");
				Serial_write(id, 2);
			}
			else if (command == 'v') {//Check Version
				CheckVersion();
			}
			else if (command == 'c') {// A 'c'calibration command
				SensorCalibration();
			}
			else if (command == 'b') {// 'b'lock erase flash
				FlashControl();
			}
			else if (command == 'p') {// Set 'p'id command
				SetPID();
			}
			else if (command == 'm') {// Set report 'm'ode
				char mode = GetChar();
				if (mode == 'e') {// Report AHRS by 'e'uler angle
					report_mode = REPORT_AHRS_EULER;
				}
				else if (mode == 'q') {// Report // Report AHRS by 'q'quaternion
					report_mode = REPORT_AHRS_QUATERNION;
				}
				else if (mode == 'r') {// Report sensor 'r'aw data
					report_mode = REPORT_SENSORS_RAW;
				}
				else if (mode == 'c') {// Report sensor 'c'alibrated data
					report_mode = REPORT_SENSORS_CALIBRATED;
				}
				else if (mode == 'm') {// Report 'm'otor power distribution
					report_mode = REPORT_MOTOR_POWER;
				}
				else if (mode == 'v') {// Report 'v'elocity
					report_mode = REPORT_VELOCITY;
				}
				else if (mode == 's') {// Report 's'tatus
					report_status();
				}
				else if (mode == 'p') {// Report controller 'p'id
					char type = GetChar();
					if (type == 'p') // 'p'id
						report_mode = REPORT_PID;
					else if (type == 'r') //'r'ate pid
						report_mode = REPORT_RATE_PID;
					else if (type == 'a') //'a'ltitude hold pid
						report_mode = REPORT_ALTHOLD_PID;
				}
			}
			else if (command == 'f') {// Set report 'f'ormat
				char format = GetChar();
				if (format == 'b') {// Report 'b'inary format
					report_format = REPORT_FORMAT_BINARY;
				}
				else if (format == 't') {// Report 't'ext format
					report_format = REPORT_FORMAT_TEXT;
				}
			}
			else if (command == 's') {// 's'tream output control
				char mode = GetChar();
				if (mode == 's') {// 's'tart stream
					stream_mode = STREAM_START;
				}
				else if (mode == 'p') {// 'p'ause stream
					stream_mode = STREAM_PAUSE;
				}
				else if (mode == 't') {// 't'oggle stream
					if(stream_mode==STREAM_START)
						stream_mode = STREAM_PAUSE;
					else
						stream_mode = STREAM_START;
				}
			}
		}
		else { 
			if (report_format == REPORT_FORMAT_TEXT) {
			printf("Unknown command.\n");
			}
		} // Skip character
	}
}

// Main Control loop
void loop()
{
	static uint32_t nextTick = 0;
	while(getTickCount()<nextTick);
	nextTick = getTickCount()+TICK_FRAME_PERIOD;
	CommandProcess();
#ifdef GPS
  GPSCommandProcess();
#endif
	SensorsRead(SENSOR_ACC|SENSOR_GYRO|SENSOR_MAG|SENSOR_BARO,1);
#ifdef OPTION_RC
	if(IsSSVConnected()) {
	ssv_rc_update();
	if(getTickCount()%1000)
		UpdateBattery();
	}
	if(ChronographRead(ChronRC)>= OUTPUT_RC_INTERVAL) {
		SensorsDynamicCalibrate(SENSOR_GYRO|SENSOR_MAG);
		ChronographSet(ChronRC);
		computeRC();
		armDetect();
	}
#endif
	if(getMagMode()||!(GetSensorCalState()&(1<<MAG)))
		nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO|SENSOR_MAG);
	else
		nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);

	if((GetFrameCount()%18)==0)
		report_sensors();
	
	IncFrameCount(1);
#ifdef OPTION_RC
	if(GetFrameCount()==MOTORS_ESC_DELAY)
		motorsStart();
	stabilizer();
	if((GetFrameCount()%12)==0)
		UpdateLED();
#endif
}

/*-----------------------------------------------------------------------------------*/
/*  Fly Controller Main Function                                                     */
/*-----------------------------------------------------------------------------------*/
int32_t main (void)
{
	setup();
	while(true) loop();
}




