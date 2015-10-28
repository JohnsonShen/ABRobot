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
#include "RC.h"
#include "ssv7241.h"

#define ROLL_BASE         (1500-1024*ROLL_SCALE/2)
#define ROLL_SCALE_BASIC   0.27f
#define ROLL_SCALE_EXPERT  0.4f
#define PITCH_BASE        (1500-1024*PITCH_SCALE/2)
#define PITCH_SCALE_BASIC  0.27f
#define PITCH_SCALE_EXPERT 0.4f
#define YAW_BASE          (1500-1024*YAW_SCALE/2)
#define YAW_SCALE          1.0f
#define THR_BASE           1070
#define THR_LOW            1200
#define THR_MID            1390
#define THR_HOV            1500
#define THR_HIGH_BASIC     1600
#define THR_HIGH_EXPERT    1800
#define THR_SCALE_BASIC    ((float)(THR_HIGH_BASIC-THR_LOW)/1024)
#define THR_SCALE_EXPERT   ((float)(THR_HIGH_EXPERT-THR_LOW)/1024)

#define AUX1_BASE          978
#define AUX1_SCALE         1.0f
#define AUX2_BASE          978
#define AUX2_SCALE         1.0f

#define FLY_MODE_BASIC    0
#define FLY_MODE_EXPERT   1

#define CHANNEL_LOW_GAP    50

uint16_t* rcValueSSV;
uint8_t	FlyMode = FLY_MODE_BASIC;
uint16_t RxChannel[RC_CHANS];

void SPI_Init(void)
{
	SYS_UnlockReg();

	CLK_EnableModuleClock(SPI0_MODULE);

	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_SPI0_CLK | SYS_GPB_MFPL_PB6MFP_SPI0_MISO0 | SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0);
	SYS->GPD_MFPL &= ~SYS_GPD_MFPL_PD3MFP_Msk;	
	SYS->GPD_MFPL |= SYS_GPD_MFPL_PD3MFP_INT1;
	
	SYS_LockReg();

	SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 1000000);   
}
void RC_SSV_Enable(char enable)
{
	if(enable)
		NVIC_EnableIRQ(EINT1_IRQn);
	else
		NVIC_DisableIRQ(EINT1_IRQn);
}
bool IsSSVReceiverConnected()
{
	if(SSV7241_CheckID()==1)
		return true;
	else
		return false;
}
bool RC_SSV_Init(void)
{
	bool beConnected;
	SPI_Init();
	beConnected = IsSSVReceiverConnected();
	if(beConnected) {
	SSV7241_Enable();	
	rcValueSSV = getValue();
	printf("\nSSV RC 2.4G Initilized.\n");
}
	return beConnected;
}
uint8_t RC_GetFlyMode()
{
	return FlyMode;
}
void RC_CheckFlyMode()
{
	if((RxChannel[ROLL_CH]<(CHANNEL_LOW_GAP<<2))&&
		(RxChannel[PITCH_CH]<(CHANNEL_LOW_GAP<<2)))
		FlyMode = FLY_MODE_EXPERT;
	else
		FlyMode = FLY_MODE_BASIC;
}
void ssv_rc_update(void)
{
	extern RF_DATA RxData,RxDataOld;
	float THR_SCALE,PITCH_SCALE,ROLL_SCALE;
	
	if(RxData.num==TxNUM) {
		if(FlyMode==FLY_MODE_EXPERT) {
			THR_SCALE = THR_SCALE_EXPERT;
			ROLL_SCALE = ROLL_SCALE_EXPERT;
			PITCH_SCALE = PITCH_SCALE_EXPERT;
		}
		else {
			THR_SCALE = THR_SCALE_BASIC;
			ROLL_SCALE = ROLL_SCALE_BASIC;
			PITCH_SCALE = PITCH_SCALE_BASIC;
		}
		
		RxChannel[THR_CH] 	= ((RxData.BUF[0]<<2)|(RxData.BUF[4]>>6)&3);
		RxChannel[ROLL_CH] 	= ((RxData.BUF[1]<<2)|(RxData.BUF[4]>>4)&3);
		RxChannel[PITCH_CH] = ((RxData.BUF[2]<<2)|(RxData.BUF[4]>>2)&3);
		RxChannel[YAW_CH] 	= ((RxData.BUF[3]<<2)| RxData.BUF[4]&3);
		RxChannel[AUX1_CH] 	= ((RxData.BUF[5]<<2)|(RxData.BUF[7]>>2)&3);
		RxChannel[AUX2_CH] 	= ((RxData.BUF[6]<<2)| RxData.BUF[7]&3);

		if(RxChannel[THR_CH]<CHANNEL_LOW_GAP)
			rcValueSSV[THR_CH] = THR_BASE;
		else
			rcValueSSV[THR_CH] = RxChannel[THR_CH]*THR_SCALE + THR_LOW;
		
		rcValueSSV[ROLL_CH] = RxChannel[ROLL_CH]*ROLL_SCALE + ROLL_BASE;
		rcValueSSV[PITCH_CH] = RxChannel[PITCH_CH]*PITCH_SCALE + PITCH_BASE;
		rcValueSSV[YAW_CH] = RxChannel[YAW_CH]*YAW_SCALE + YAW_BASE;
		rcValueSSV[AUX1_CH] = RxChannel[AUX1_CH]*AUX1_SCALE + AUX1_BASE;
		rcValueSSV[AUX2_CH] = RxChannel[AUX2_CH]*AUX2_SCALE + AUX2_BASE;
		RxData.num=0;
	}
}
uint16_t readSsvRC(uint8_t chan) 
{
	uint16_t data;

	data = rcValueSSV[chan]; 

	return data;
}
int16_t GetSSVThrust()
{
	return ((readSsvRC(THR_CH) - THR_LOW) + (THR_HOV - THR_MID));
}
int16_t GetSSVAltitude()
{
	return (readSsvRC(THR_CH) - THR_MID);
}
