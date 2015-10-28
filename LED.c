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
#include "LED.h"
#include "Def.h"
#include "stabilizer.h"
#include "Sensors.h"
#include "Battery.h"
#include "AltHold.h"
#include "ssv7241.h"
#include "RC_ssv.h"
#ifdef M451
#if (BOARD_CODE == 200)
#define IO_STATE_ARM        PB, BIT0	//PA, BIT0
#define IO_ARM              PB0				//PA0
#define IO_STATE_HEAD_MODE  PB, BIT8	//PA, BIT1
#define IO_MAG              PB8
#else
#define IO_STATE_ARM        PA, BIT0
#define IO_ARM              PA0
#define IO_STATE_HEAD_MODE  PA, BIT1
#define IO_MAG              PA1
#endif
#define IO_STATE_HFREE_MODE PA, BIT2
#define IO_HFREE_MODE       PA2
#else
#define IO_STATE_ARM        E_GPB, 8
#endif  
static char ledState = 0;
char GetLedState()
{
	return ledState;
}
void LED_Init(void)
{
	#ifdef M451
	GPIO_SetMode(IO_STATE_ARM, GPIO_MODE_OUTPUT);
	GPIO_SetMode(IO_STATE_HEAD_MODE, GPIO_MODE_OUTPUT);
	GPIO_SetMode(IO_STATE_HFREE_MODE, GPIO_MODE_OUTPUT);
	IO_ARM = 1;
	IO_MAG = 1;
	IO_HFREE_MODE = 1;
#else
	DrvGPIO_Open(IO_STATE_ARM, E_IO_OUTPUT);
	DrvGPIO_SetBit(IO_STATE_ARM);
#endif	
}

void led_arm_state(char state)
{
	if(state==LED_STATE_TOGGLE)
		ledState=ledState^(1<<LED_ARM);	
	else if(state==LED_STATE_ON)
		ledState|=(1<<LED_ARM);
	else if(state==LED_STATE_OFF)
		ledState&=~(1<<LED_ARM);	
}
void led_mag_state(char state)
{
	if(state==LED_STATE_TOGGLE)
		ledState=ledState^(1<<LED_MAG);	
	else if(state==LED_STATE_ON)
		ledState|=(1<<LED_MAG);
	else if(state==LED_STATE_OFF)
		ledState&=~(1<<LED_MAG);	
}

void UpdateLED()
{
	if((GetSensorCalState()&(1<<MAG))) {
		if(getMagMode())
			ledState|=(1<<LED_MAG);
		else 
			ledState&=~(1<<LED_MAG);
	}
	
	if(getHeadFreeMode())
		ledState|=(1<<LED_HEAD_FREE);
	else
		ledState&=~(1<<LED_HEAD_FREE);
#if STACK_BARO
	if(GetAltHoldMode())
		ledState|=(1<<LED_ALTHOLD);
	else
#endif
		ledState&=~(1<<LED_ALTHOLD);
	
	if(CheckLowBattery())
		ledState|=(1<<LED_LOW_BAT);
	else
		ledState&=~(1<<LED_LOW_BAT);
	
	if(CheckLowRSSI())
		ledState|=(1<<LED_LOW_RSSI);
	else
		ledState&=~(1<<LED_LOW_RSSI);
	
	if(RC_GetFlyMode())
		ledState|=(1<<LED_FLY_MODE);
	else
		ledState&=~(1<<LED_FLY_MODE);

	if((ledState&(1<<LED_ARM))==0)
#ifdef M451
		IO_ARM = 1;
#else
		DrvGPIO_SetBit(IO_STATE_ARM);
#endif
	else
#ifdef M451
		IO_ARM = 0;
#else
		DrvGPIO_ClrBit(IO_STATE_ARM);
#endif
	
#ifdef M451
	if((ledState&(1<<LED_MAG))==0)
		IO_MAG = 1;
	else
		IO_MAG = 0;
#endif
	
#ifdef M451
	if((ledState&(1<<LED_HEAD_FREE))==0)
		IO_HFREE_MODE = 1;
	else
		IO_HFREE_MODE = 0;
#endif
}
