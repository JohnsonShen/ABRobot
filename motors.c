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
 * [2015/07/16] CP Lu                                                         *
 * 1. Modify motorsSetRatio() to increase set ration speed                    *
 * 2. Modify PWM_ConfigOutputChannelf(). Change PWM timer from down count to  *
 *    up count. Fixd waveform control setting's type *                        *
 *============================================================================*
 */
#ifdef M451
#include "M451Series.h"
#else
#include "Driver\DrvGPIO.h"
#include "Driver\DrvPWM.h"
#endif
#include "Def.h"
#include "motors.h"

uint8_t volatile g_u8PWMCount = 1;
uint16_t g_u16Frequency;

#ifdef M451
uint32_t PWM_ConfigOutputChannelf(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Frequency, float u32DutyCycle)
{
	uint32_t u32Src;
	uint32_t u32PWMClockSrc;
	uint32_t i;
	uint16_t u16Prescale = 1, u16CNR = 0xFFFF;

	if(pwm == PWM0)
		u32Src = CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk;
	else//(pwm == PWM1)
		u32Src = CLK->CLKSEL2 & CLK_CLKSEL2_PWM1SEL_Msk;

	if(u32Src == 0) {
		//clock source is from PLL clock
			u32PWMClockSrc = CLK_GetPLLClockFreq();
	} 
	else {
		//clock source is from PCLK
		SystemCoreClockUpdate();
		u32PWMClockSrc = SystemCoreClock;
	}

	for(u16Prescale = 1; u16Prescale < 0xFFF; u16Prescale++) {
		i = (u32PWMClockSrc / u32Frequency) / u16Prescale;
		if(i > (0x10000))
			continue;
		u16CNR = i;
		break;
	}

	i = u32PWMClockSrc / ((u16Prescale + 1) * u16CNR);
	PWM_SET_PRESCALER(pwm, u32ChannelNum, --u16Prescale);
    /* Set Counter to upcount */
	(pwm)->CTL1 = ((pwm)->CTL1 & ~(PWM_CTL1_CNTTYPE0_Msk << (2 * u32ChannelNum))) | (0UL << (2 * u32ChannelNum));
	(pwm)->CTL1 &= ~(PWM_CTL1_CNTMODE0_Msk << u32ChannelNum);

    if(u32DutyCycle == 100)
    {
        PWM_SET_CMR(pwm, u32ChannelNum, u16CNR);
        PWM_SET_CNR(pwm, u32ChannelNum, --u16CNR);        
    }
    else
    {
	PWM_SET_CMR(pwm, u32ChannelNum, (uint32_t)(u32DutyCycle * (u16CNR + 1) / 100 - 1));
	PWM_SET_CNR(pwm, u32ChannelNum, --u16CNR);
    }

	(pwm)->WGCTL0 = ((pwm)->WGCTL0 & ~(PWM_WGCTL0_ZPCTL0_Msk << (u32ChannelNum * 2 + PWM_WGCTL0_ZPCTL0_Msk)))
                    | (PWM_OUTPUT_HIGH << (u32ChannelNum * 2 + PWM_WGCTL0_ZPCTL0_Pos));
	(pwm)->WGCTL1 = ((pwm)->WGCTL1 & ~(PWM_WGCTL1_CMPUCTL0_Msk << (u32ChannelNum * 2 + PWM_WGCTL1_CMPUCTL0_Pos)))
                    | (PWM_OUTPUT_LOW << (u32ChannelNum * 2 + PWM_WGCTL1_CMPUCTL0_Pos));
	return(i);
}
#else
static uint32_t s_u32Pulse = 0;
static S_DRVPWM_TIME_DATA_T sPt;
/*---------------------------------------------------------------------------------------------------------*/
/* PWM Timer Callback function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void DRVPWM_PwmIRQHandler()
{
	if(s_u32Pulse == 1 * g_u16Frequency /10) {
		DrvPWM_SetTimerCounter(DRVPWM_TIMER0, 0);
	}
	
	if (s_u32Pulse == 1 * g_u16Frequency /10 + 1)
		g_u8PWMCount = 0;

	s_u32Pulse++;   
} 
#endif
void motorsStart()
{
#ifdef M451
	PWM_Start(PWM0, PWM_CH_0_MASK);
	PWM_Start(PWM0, PWM_CH_1_MASK);
	PWM_Start(PWM0, PWM_CH_2_MASK);
	PWM_Start(PWM0, PWM_CH_3_MASK);
#ifdef HEX6X 
	PWM_Start(PWM0, PWM_CH_4_MASK);
	PWM_Start(PWM0, PWM_CH_5_MASK);
#endif
#else
	DrvPWM_Enable(DRVPWM_TIMER0, 1);
	DrvPWM_Enable(DRVPWM_TIMER1, 1);
	DrvPWM_Enable(DRVPWM_TIMER2, 1);
	DrvPWM_Enable(DRVPWM_TIMER3, 1);
	#ifdef HEX6X 
	DrvPWM_Enable(DRVPWM_TIMER4, 1);
	DrvPWM_Enable(DRVPWM_TIMER5, 1);
#endif
#endif
}
void motorsInit()
{
#ifdef M451
	CLK_EnableModuleClock(PWM0_MODULE);
	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

	SYS->GPC_MFPL = (SYS->GPC_MFPL 
	& (~SYS_GPC_MFPL_PC0MFP_Msk)
	& (~SYS_GPC_MFPL_PC1MFP_Msk)
	& (~SYS_GPC_MFPL_PC2MFP_Msk)
	& (~SYS_GPC_MFPL_PC3MFP_Msk)
#ifdef HEX6X 
	& (~SYS_GPC_MFPL_PC4MFP_Msk)
#endif
	);
#ifdef HEX6X 
	SYS->GPD_MFPL = (SYS->GPD_MFPL 
	& (~SYS_GPD_MFPL_PD7MFP_Msk)
	);
#endif
	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_PWM0_CH0|SYS_GPC_MFPL_PC1MFP_PWM0_CH1|
	SYS_GPC_MFPL_PC2MFP_PWM0_CH2|SYS_GPC_MFPL_PC3MFP_PWM0_CH3
#ifdef HEX6X 
	|SYS_GPC_MFPL_PC4MFP_PWM0_CH4
#endif
	);
#ifdef HEX6X 
	SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD7MFP_PWM0_CH5);
#endif
	PWM_ConfigOutputChannelf(PWM0, 0, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
	PWM_ConfigOutputChannelf(PWM0, 1, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
	PWM_ConfigOutputChannelf(PWM0, 2, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
	PWM_ConfigOutputChannelf(PWM0, 3, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
#ifdef HEX6X 
	PWM_ConfigOutputChannelf(PWM0, 4, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
	PWM_ConfigOutputChannelf(PWM0, 5, ESC_UPDATE_FREQ, ESC_UPDATE_FREQ/10);
#endif

	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
#ifdef HEX6X 
	motorsSetRatio(MOTOR_M5, 0);
	motorsSetRatio(MOTOR_M6, 0);
#endif

	PWM_EnableOutput(PWM0, PWM_CH_0_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_1_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_2_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);
#ifdef HEX6X 
	PWM_EnableOutput(PWM0, PWM_CH_4_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_5_MASK);
#endif
#else
	uint8_t u8Timer;

	DrvPWM_Open();
	DrvGPIO_InitFunction(E_FUNC_PWM01);
	DrvGPIO_InitFunction(E_FUNC_PWM23);

	s_u32Pulse = 0;
	g_u8PWMCount = 1;
	g_u16Frequency = ESC_UPDATE_FREQ;
	sPt.u8Mode = DRVPWM_AUTO_RELOAD_MODE;
	sPt.u32Frequency = g_u16Frequency;
	sPt.u8HighPulseRatio = 1;
	sPt.u8HighPulseBase = 1000000/g_u16Frequency;
	sPt.i32Inverter = 0;

	u8Timer = DRVPWM_TIMER0;
	DrvPWM_SelectClockSource(u8Timer, DRVPWM_HCLK);
	DrvPWM_SetTimerClk(u8Timer, &sPt);
	DrvPWM_SetTimerIO(u8Timer, 1);

	u8Timer = DRVPWM_TIMER1;
	DrvPWM_SelectClockSource(u8Timer, DRVPWM_HCLK);
	DrvPWM_SetTimerClk(u8Timer, &sPt);
	DrvPWM_SetTimerIO(u8Timer, 1);

	u8Timer = DRVPWM_TIMER2;
	DrvPWM_SelectClockSource(u8Timer, DRVPWM_HCLK);
	DrvPWM_SetTimerClk(u8Timer, &sPt);
	DrvPWM_SetTimerIO(u8Timer, 1);

	u8Timer = DRVPWM_TIMER3;
	DrvPWM_SelectClockSource(u8Timer, DRVPWM_HCLK);
	DrvPWM_SetTimerClk(u8Timer, &sPt);
	DrvPWM_SetTimerIO(u8Timer, 1);
#endif
}

void motorsSetRatio(int id, uint16_t u16PulseWidth)
{
#ifdef M451
    uint32_t u32CNR = 0, u32CMR = 0;
    
    u32CNR = PWM_GET_CNR(PWM0, id);
    if(u16PulseWidth >= MAX_PULSE_WIDTH)
        u32CMR = u32CNR+1;
    else if(u16PulseWidth == 0)
        u32CMR = 0;
    else
        u32CMR = (float)u16PulseWidth*ESC_FREQ_UPDATE_FREQ_DIV_1000000* u32CNR;    /* Duty cycle(%)= ratio*ESC_UPDATE_FREQ/1000000 */

    PWM_SET_CMR(PWM0, id, u32CMR);
    
	//PWM_ConfigOutputChannelf(PWM0, id, ESC_UPDATE_FREQ, (float)ratio*ESC_UPDATE_FREQ/10000);/*ratio*100*490/1000000*/
#else
	uint8_t u8Timer;

	switch(id) {
		case MOTOR_M1:
			u8Timer = DRVPWM_TIMER0;
			break;
		case MOTOR_M2:
			u8Timer = DRVPWM_TIMER1;
			break;
		case MOTOR_M3:
			u8Timer = DRVPWM_TIMER2;
			break;
		case MOTOR_M4:
			u8Timer = DRVPWM_TIMER3;
			break;
		default :
		u8Timer = 0;
	}

	sPt.u8HighPulseRatio = ratio;
	DrvPWM_SetTimerClk(u8Timer, &sPt);
#endif
}
