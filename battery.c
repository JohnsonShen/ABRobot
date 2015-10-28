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
#define BAT_FULL 	12.5f
#define BAT_LOW		10.8f
#define BAT_TH		15
#ifdef M451
#include "M451Series.h"
#else
#include "DrvGPIO.h"
#endif
#include "battery.h"
#include "Def.h"

#ifdef M451
#define IO_STATE_BAT	PB, BIT2
#define IO_BAT        PB2
#else
#endif  

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
static uint16_t  AdcData;
static int8_t BatteryPercent=0;
/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ADC00_IRQHandler(void)
{
	g_u32AdcIntFlag = 1;
	EADC_CLR_INT_FLAG(EADC, 0x1);      /* Clear the A/D ADINT0 interrupt flag */
}
void Battery_Init(void)
{
#ifdef M451
	SYS_UnlockReg();
	/* Enable EADC module clock */
	CLK_EnableModuleClock(EADC_MODULE);	
	/* EADC clock source is 72MHz, set divider to 8, ADC clock is 72/8 MHz */
	CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));
	SYS_LockReg();	
	/* Configure the GPB0 - GPB3 ADC analog input pins.  */
	SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
	SYS->GPB_MFPL |= SYS_GPB_MFPL_PB2MFP_EADC_CH2;
	
	GPIO_DISABLE_DIGITAL_PATH(PB, 0x4);
	
	/* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
	EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
	EADC_SetInternalSampleTime(EADC, 6);

	/* Configure the sample module 0 for analog input channel 2 and software trigger source.*/
	EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 2);
	
	/* Clear the A/D ADINT0 interrupt flag for safe */
	EADC_CLR_INT_FLAG(EADC, 0x1);

	/* Enable the sample module 0 interrupt.  */
	EADC_ENABLE_INT(EADC, 0x1);//Enable sample module A/D ADINT0 interrupt.
	EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);//Enable sample module 0 interrupt.
	
	/* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
	g_u32AdcIntFlag = 0;
	
	/* Enable battery detect circuit (PA3=1)*/
	GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
	PA3=1;
#else
	/* TBD.. */
#endif	
}
bool CheckLowBattery()
{
	if(GetBattery()<BAT_TH)
		return true;
	else
		return false;
}
void UpdateBattery()
{
	EADC_START_CONV(EADC, 0x1);
	AdcData = EADC_GET_CONV_DATA(EADC, 0);
}
uint8_t GetBattery()
{
	int8_t CurrentBattery;
	CurrentBattery = ((AdcData-2700)*100/(3240-2700));
	BatteryPercent = BatteryPercent*0.95f + CurrentBattery*0.05f;
	//BatteryPercent = CurrentBattery;
	if(BatteryPercent<0)
		BatteryPercent = 0;
	else if(BatteryPercent>100)
		BatteryPercent = 100;
		
	return BatteryPercent;
}
