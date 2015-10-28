/*============================================================================*
 * SSV7241 FHSS  Rx                                                           *
 * version:  V1.2                                                             *
 * Written by Jliu for Nuvoton Technology.                                    *
 * jliu@nuvoton.com                                                           *
 *                                                                            *
 *============================================================================*/
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
#endif
#include "ssv7241.h"
#include "Timer_Ctrl.h"
#include "LED.h"
#include "Timer_Ctrl.h"
#include "FlashCtrl.h"
#include "Report.h"
//#define V12
#define MATCH_ADDRESS_INVALID_COUNT 1000 /* 8 second */
#define RC_LOOSE_COUNT 50 /* 8 second */
#define SYS_Delay_10us() Delay1us(10)
#define high_tx_pwr 1
#define simplified  0

/* default TX/RX Address and Address Width */
uint8 TX_ADDRESS[TX_ADR_WIDTH] = {0xE3,0xE7,0xE7,0xE7,0xE7}; 
uint8 RX_ADDRESS[RX_ADR_WIDTH] = {0xE3,0xE7,0xE7,0xE7,0xE7}; 
uint8 MATCH_ADDRESS[RX_ADR_WIDTH];
bool  MATCH_ADDRESS_VALID = false;
bool  MATCH_ADDRESS_PROCESS = false;
bool  SSV_RC_CONNECT = false;
uint16_t NO_ACK_SEQUENCE_COUNT = 0;

const uint8_t ChannelBit[8]   = {0x1,0x2,0x4,0x8,0x10,0x20,0x40,0x80};
const uint8_t ChannelTable[8] = {77,7,75,43,73,45,70,68};

volatile uint8_t channel;
volatile uint8_t ChannelFlag;
volatile uint8_t RxState;
volatile RF_DATA RxData,RxDataOld;
volatile uint8_t NUM;

uint8 DYN_ACKOn = 0;
uint8 ACK_PAYOn = 0;
float RSSIf=0;
int SSV_TICKCNT=0;
#if simplified 
uint8 Initial_Reg_Array[] = {
0x1F, 0x01, 0x00,
0x1B, 0x04, 0xD0, 0xE1, 0xD3, 0x3D,
0x19, 0x04, 0x06, 0xAA, 0x62, 0xC3,
0x1F, 0x01, 0x01,
#if high_tx_pwr
0x18, 0x04, 0xBE, 0x6B, 0x96, 0x5B,
#else
0x18, 0x04, 0x5C, 0x14, 0x00, 0x5B,
#endif
0x19, 0x04, 0x77, 0x48, 0x9A, 0xC8,
0x1B, 0x04, 0x76, 0x87, 0xCA, 0x01,
0x1F, 0x01, 0x02,
0x1B, 0x04, 0xA0, 0x00, 0x18, 0xA0,
0x1F, 0x01, 0x04,
0x18, 0x04, 0x01, 0x00, 0xF0, 0x00,
0x1F, 0x01, 0x05,
0x18, 0x04, 0x84, 0x03, 0x2A, 0x03,
0x19, 0x04, 0x90, 0xBF, 0x00, 0x00,
0x1A, 0x04, 0xA0, 0x0F, 0x00, 0x00,
0x1F, 0x01, 0x00
};
#else
#ifndef V12
uint8 Initial_Reg_Array[] = {
0x1F, 0x01, 0x00,
0x1B, 0x04, 0xD0, 0xE1, 0xD3, 0x3D,
0x19, 0x04, 0x06, 0xAA, 0x62, 0xC3,
0x1F, 0x01, 0x01,
#if high_tx_pwr
0x18, 0x04, 0xBE, 0x6B, 0x96, 0x5B,
#else
0x18, 0x04, 0x5C, 0x14, 0x00, 0x5B,
#endif
0x19, 0x04, 0x77, 0x48, 0x9A, 0xC8,
0x1B, 0x04, 0x76, 0x87, 0xCA, 0x01,
0x1F, 0x01, 0x02,
0x1B, 0x04, 0xA0, 0x00, 0x18, 0xA0,
0x1F, 0x01, 0x04,
0x18, 0x04, 0x01, 0x00, 0xF0, 0x00,
0x1F, 0x01, 0x05,
0x18, 0x04, 0x84, 0x03, 0x2A, 0x03,
0x19, 0x04, 0x90, 0xBF, 0x00, 0x00,
0x1A, 0x04, 0xA0, 0x0F, 0x00, 0x00,
0x1F, 0x01, 0x08,
0x19, 0x01, 0x10,
0x18, 0x04, 0x00, 0xF8, 0xFF, 0x03,
0x19, 0x01, 0x9E,
0x19, 0x01, 0xBE,
0x18, 0x04, 0x07, 0x02, 0x04, 0x00,
0x19, 0x01, 0xDE,
0x19, 0x01, 0xFE,
0x18, 0x04, 0x00, 0x7C, 0x44, 0x00,
0x19, 0x01, 0x9F,
0x19, 0x01, 0xBF,
0x18, 0x04, 0x07, 0x02, 0x04, 0x00,
0x19, 0x01, 0xDF,
0x19, 0x01, 0xFF,
0x18, 0x04, 0x11, 0xFE, 0xFF, 0x00,
0x19, 0x01, 0x1E,
0x19, 0x01, 0x3E,
0x18, 0x04, 0x01, 0x8B, 0x30, 0x08,
0x19, 0x01, 0x5E,
0x19, 0x01, 0x7E,
0x18, 0x04, 0x10, 0xFF, 0x85, 0x00,
0x19, 0x01, 0x1F,
0x19, 0x01, 0x3F,
0x18, 0x04, 0x01, 0x8B, 0x30, 0x0C,
0x19, 0x01, 0x5F,
0x19, 0x01, 0x7F,
0x19, 0x01, 0x00,
0x1F, 0x01, 0x09,
0x18, 0x04, 0x00, 0x00, 0xF4, 0xF8,
0x1F, 0x01, 0x00
};
#else
uint8 Initial_Reg_Array[] = {
0x1F, 0x01, 0x00,
0x1B, 0x04, 0x10, 0xE1, 0xD3, 0x3D,
0x19, 0x04, 0x06, 0xAA, 0xA2, 0xDB,
0x1F, 0x01, 0x01,
0x19, 0x04, 0x77, 0x48, 0x9A, 0xE8,
0x1B, 0x04, 0x76, 0x87, 0xCA, 0x01,
0x1F, 0x01, 0x02,
0x1B, 0x04, 0xA0, 0x00, 0x18, 0xA0,
0x1F, 0x01, 0x04,
0x18, 0x04, 0x01, 0x00, 0xF0, 0x00,
0x1F, 0x01, 0x05,
0x18, 0x04, 0x84, 0x03, 0x2A, 0x03,
0x19, 0x04, 0x90, 0xBF, 0x00, 0x00,
0x1A, 0x04, 0xA0, 0x0F, 0x00, 0x00,
0x1F ,0x01 ,0x08,
0x19 ,0x01 ,0x10,
0x18 ,0x04 ,0x07 ,0x02 ,0x04 ,0x00,
0x19 ,0x01 ,0xD8,
0x19 ,0x01 ,0xF8,
0x18 ,0x04 ,0xD1 ,0x01 ,0xF8 ,0x03,
0x19 ,0x01 ,0x9E,
0x19 ,0x01 ,0xBE,
0x18 ,0x04 ,0x00 ,0x0A ,0x08 ,0x00,
0x19 ,0x01 ,0xDE,
0x19 ,0x01 ,0xFE,
0x18 ,0x04 ,0xCE ,0xC1 ,0xFF ,0x03,
0x19 ,0x01 ,0x9F,
0x19 ,0x01 ,0xbF,
0x18 ,0x04 ,0x00 ,0x08 ,0x08 ,0x00,
0x19 ,0x01 ,0xDF,
0x19 ,0x01 ,0xFF,
0x18 ,0x04 ,0x1D ,0xEE ,0xEF ,0x00,
0x19 ,0x01 ,0x1E,
0x19 ,0x01 ,0x3E,
0x18 ,0x04 ,0x07 ,0x5B ,0x30 ,0x0A,
0x19 ,0x01 ,0x5E,
0x19 ,0x01 ,0x7E,
0x18 ,0x04 ,0x66 ,0xCC ,0xFC ,0x00,
0x19 ,0x01 ,0x1F,
0x19 ,0x01 ,0x3F,
0x18 ,0x04 ,0x01 ,0x0B ,0x50 ,0x0C,
0x19 ,0x01 ,0x5F,
0x19 ,0x01 ,0x7F,
0x19 ,0x01 ,0x25,
0x1F ,0x01 ,0x09,
0x18 ,0x04 ,0x00 ,0x00 ,0xFE ,0xFE,
0x1F ,0x01 ,0x00
};
#endif
#endif

void Delay1us(uint32_t CNT)
{
	while(CNT--);
}
bool IsSSVRCConnected()
{
	return SSV_RC_CONNECT;
}
bool GetMatchAddressProcess()
{
	return MATCH_ADDRESS_PROCESS;
}
void GetMatchAddress(uint8_t* RX_ADDRESS)
{
	int8_t i;
	for(i=0;i<RX_ADR_WIDTH;i++)
		RX_ADDRESS[i] = MATCH_ADDRESS[i];
}
void SetMatchAddress(uint8_t* RX_ADDRESS)
{
	int8_t i;
	for(i=0;i<RX_ADR_WIDTH;i++)
		MATCH_ADDRESS[i] = RX_ADDRESS[i];
}
bool CheckLowRSSI()
{
	if(RxData.rssi==0)
		return true;
	else
		return false;
}
uint8_t GetRSSI()
{
	return RxData.rssi;
}
float GetRSSIf()
{
	return RSSIf;
}
void SSV7241_Init()
{
	uint8 value; 
	uint32 i = 0, j = 0, k = 0;
	uint8 data[4];
	uint32 array_length = sizeof(Initial_Reg_Array);

	while(i < array_length) {
		if(Initial_Reg_Array[i+1] == 0x01) {
			SSV7241_Write_Reg( W_REG | Initial_Reg_Array[i], Initial_Reg_Array[i+2]);
			i += 3;
		}
		else { 
			k=Initial_Reg_Array[i+1];
			
			for(j = 0; j < k; j++) {
				data[j] = Initial_Reg_Array[i+2+j];
			}

			SSV7241_Write_Buf( W_REG | Initial_Reg_Array[i], data, Initial_Reg_Array[i + 1]);
			i += 2+k;
		}
	}
	
	value = SSV7241_Read_Reg(CFG_TOP); 
	value |= (EN_CRC | CRC_2B);
	SSV7241_Write_Reg(W_REG | CFG_TOP, value);

	if(GetFlashRxAddress(MATCH_ADDRESS)==false) {
		SetMatchAddress(RX_ADDRESS);
		MATCH_ADDRESS_PROCESS = true;
		MATCH_ADDRESS_VALID = false;
	}
	else
		MATCH_ADDRESS_PROCESS = false;
	if (report_format == REPORT_FORMAT_TEXT) {
	printf("MATCH_ADDRESS:%x %x %x %x %x\n",MATCH_ADDRESS[0],MATCH_ADDRESS[1],
							MATCH_ADDRESS[2],MATCH_ADDRESS[3],MATCH_ADDRESS[4]);
	printf("MATCH_ADDRESS_PROCESS:%d\n",MATCH_ADDRESS_PROCESS);
	}
	SSV7241_Write_Buf(W_REG | TX_ADDR,MATCH_ADDRESS,5);
	SSV7241_Write_Buf(W_REG | RX_ADDR_P0,MATCH_ADDRESS,5);	
	
	SSV7241_Write_Reg(W_REG | EN_AA,0x01);    
	SSV7241_Write_Reg(W_REG | EN_RXADDR,0x01);
	SSV7241_Write_Reg(W_REG | RF_CH,77);	
	SSV7241_Write_Reg(W_REG | RX_PW_P0,1);
	SSV7241_Enable_dynamic_payload(DPL_P0);	
		
	SSV7241_Feature_ACK_PAY(1);
	SSV7241_Write_Reg(W_REG | SETUP_RETR,0);
	SSV7241_Write_Reg(W_REG | SETUP_RETR,ARD_750us|ARC_2);	

	value = SSV7241_Read_Reg(SETUP_RF);
	value &= ~(RATE_2M|RATE_250K);
	value |= RATE_250K;
	value &= ~(PWR_LEVEL4);
	value |= PWR_LEVEL4;
	SSV7241_Write_Reg(W_REG | SETUP_RF, value); 
}

uint8 SSV7241_CheckID(void)
{
	uint8 ret = 0;
	uint8 data[4];
	
	/* When power off, Chip ID can be read */
	SSV7241_PowerOff();
	/* When RSSI disable, Chip ID can be read */
	SSV7241_DisableRSSI();
	
	SSV7241_Read_Buf(0x18, data, 4); 
	
	if(data[0] == 0x41 && data[1] == 0x72)
		ret = 1;

	return ret;
}

void SSV7241_Write_Reg(uint8 reg,uint8 value)
{
	CS_Low;
	SPI0_communication(reg);
	SPI0_communication(value);
	CS_High;   
}

uint8 SSV7241_Read_Reg(uint8 reg)
{
	uint8 value;

	CS_Low;	    
	SPI0_communication(reg);
	value = SPI0_communication(NOP);
	CS_High; 

	return value;
}

void SSV7241_Read_Buf(uint8 reg,uint8 *pBuf,uint8 len)
{
	uint8 i;
	
	CS_Low;
	SPI0_communication(reg);

	for(i = 0; i < len; i++)
		pBuf[i]=SPI0_communication(0XFF);
	
	CS_High; 
}

void SSV7241_Write_Buf(uint8 reg, uint8 *pBuf, uint8 len)
{
	uint8 i;
	
	CS_Low;
	SPI0_communication(reg);
	
	for(i = 0; i < len; i++)
		SPI0_communication(*pBuf++); 
	
	CS_High;	
}

uint8 SSV7241_isRXReady(void)
{
	uint8 ret = 0, state;
	
	state = SSV7241_Read_Reg(STATUS);  
	if(state & RX_DR)
		ret = 1;
	
	return ret;
}

uint8 SSV7241_isTXReady(void)
{
	uint8 ret = 0, state;
	
	state = SSV7241_Read_Reg(STATUS);  
	if(state & TX_DS)
		ret = 1;
	
	return ret;
}

uint8 SSV7241_isTXFULL(void)
{
	uint8 ret = 0, state;

	state = SSV7241_Read_Reg(STATUS_FIFO);
	if(state & TX_FULL)
		ret = 1;
	
	return ret;
}

uint8 SSV7241_isRXEMPTY(void)
{
	uint8 ret = 0, state;
	
	state = SSV7241_Read_Reg(STATUS_FIFO);
	if(state & RX_EMPTY)
		ret = 1;
	return ret;	
}

void SSV7241_ClearRX_DR(void)
{
	uint8 state;

	state = SSV7241_Read_Reg(STATUS);
	state |= RX_DR;
	SSV7241_Write_Reg(W_REG | STATUS, state);
}

void SSV7241_ClearTX_DS(void)
{
	uint8 state;
	
	state = SSV7241_Read_Reg(STATUS); 
	state |= TX_DS;	
	SSV7241_Write_Reg(W_REG | STATUS, state); 
}

void SSV7241_ClearMAX_RT(void)
{
	uint8 state;
	
	state = SSV7241_Read_Reg(STATUS); 
	state |= MAX_RT;	
	SSV7241_Write_Reg(W_REG | STATUS, state);
}

void SSV7241_FlushRX(void)
{
	SSV7241_Write_Reg(FLUSH_RX, NOP);
}

void SSV7241_FlushTX(void)
{
	SSV7241_Write_Reg(FLUSH_TX, NOP);
}

uint8 SSV7241_RxPacket(uint8 *rxbuf)
{
	uint8 ret = 0, len = 0;

	if(SSV7241_isRXReady()) {
		len = SSV7241_Read_Reg(R_RX_PL_WID);
		SSV7241_Read_Buf(R_RX_PLOAD, rxbuf, len); 
		SSV7241_ClearRX_DR();
		SSV7241_FlushRX();
		ret = len;
	}
	
	return ret;
}

void SSV7241_TxPacket(uint8 *txbuf, uint8 len)
{
	//Check Queue is full or not and Send packet
	if(!SSV7241_isTXFULL()) {
		SSV7241_FlushTX();
		
		if(!DYN_ACKOn)
			SSV7241_Write_Buf(W_TX_PLOAD, txbuf, len);
		else
			SSV7241_Write_Buf(W_TX_PLOAD_NOACK, txbuf, len);
		
		CE_High;
		SYS_Delay_10us();
		CE_Low;		
	}
}

void SSV7241_TxACKPacket(uint8 *txbuf, uint8 len)
{ 
	//Check Queue is full or not and Send packet
	SSV7241_FlushTX();
	SSV7241_ClearTX_DS();	
	SSV7241_Write_Buf(W_ACK_PLOAD, txbuf, len);
}

void SSV7241_TxPacketWithoutAck(uint8 *txbuf, uint8 len)
{ 
	//Check Queue is full or not and Send packet
	if(!SSV7241_isTXFULL()) {

		if(DYN_ACKOn)
			SSV7241_Write_Buf(W_TX_PLOAD, txbuf, len);
		else
			SSV7241_Write_Buf(W_TX_PLOAD_NOACK, txbuf, len);
		
		CE_High;
		SYS_Delay_10us();
		CE_Low;
	}
}	

void SSV7241_RX_Mode(void)
{
	uint8 value;
	
	value = SSV7241_Read_Reg(CFG_TOP); 
	value |= RX_ON;	
	SSV7241_Write_Reg(W_REG | CFG_TOP, value); 	
	CE_High;
}

void SSV7241_TX_Mode(void)
{
	uint8 value;
	
	value = SSV7241_Read_Reg(CFG_TOP); 
	value &= ~RX_ON;
	SSV7241_Write_Reg(W_REG | CFG_TOP, value); 
	
}

void SSV7241_PowerOn(void)
{
	uint8 value;
	uint8 data[4];
#ifndef V12
	value = SSV7241_Read_Reg(CFG_TOP); 
	value |= PWR_ON;	
	SSV7241_Write_Reg(W_REG | CFG_TOP, value); 
	
	/* refine RF */
	data[0] = 0x06;
	data[1] = 0xAA;
	data[2] = 0x62;
	data[3] = 0xC3;
	SSV7241_Write_Buf( W_REG | 0x19, data, 4);
#else
	/* refine RF */
	data[0] = 0x06;
	data[1] = 0xAA;
	data[2] = 0xA2;
	data[3] = 0xDB;
	SSV7241_Write_Buf( W_REG | 0x19, data, 4);
	value = SSV7241_Read_Reg(CFG_TOP); 
	value |= PWR_ON;	
	SSV7241_Write_Reg(W_REG | CFG_TOP, value); 
#endif
}

void SSV7241_PowerOff(void)
{
	uint8 value;
	uint8 data[4];
	
	value = SSV7241_Read_Reg(CFG_TOP); 
	value &= ~PWR_ON;
	SSV7241_Write_Reg(W_REG | CFG_TOP, value); 	
	
	/* refine RF */
	data[0] = 0x06;
	data[1] = 0xAA;
	data[2] = 0x62;
	data[3] = 0x43;
	SSV7241_Write_Buf( W_REG | 0x19, data, 4);
}

void SSV7241_Feature_DYN_ACK(uint8 Dyn_ack)
{
	uint8 value;
	
	/* Dynamic Ack Setting */
	value = SSV7241_Read_Reg(FEATURE);
	
	if(Dyn_ack) {
		value |= EN_DYN_ACK;
		DYN_ACKOn = 1;
	}
	else {
		value &= ~EN_DYN_ACK;
		DYN_ACKOn = 0;
	}
	
	SSV7241_Write_Reg(W_REG | FEATURE, value); 	
}

void SSV7241_Feature_DPL(uint8 Dpl)
{
	uint8 value;
	
	/* Dynamic Ack Setting */
	value = SSV7241_Read_Reg(FEATURE);
	
	if(Dpl)
		value |= EN_DPL;
	else
		value &= ~EN_DPL;
	
	SSV7241_Write_Reg(W_REG | FEATURE, value); 	
}

void SSV7241_Feature_ACK_PAY(uint8 Ack_pay)
{
	uint8 value;
	
	/* Ack Payload Setting */
	value = SSV7241_Read_Reg(FEATURE);
	
	if(Ack_pay) {
		value |= EN_ACK_PAY;
		ACK_PAYOn = 1;
	}
	else {
		value &= ~EN_ACK_PAY;
		ACK_PAYOn = 0;
	}
	
	SSV7241_Write_Reg(W_REG | FEATURE, value); 	
}

void SSV7241_EnableRSSI(void)
{
	uint8 value;
	
	value = SSV7241_Read_Reg(RSSI);
	value |= EN_RSSI;
	
	SSV7241_Write_Reg(W_REG | RSSI, value); 	
}

void SSV7241_DisableRSSI(void)
{
	uint8 value;
	
	value = SSV7241_Read_Reg(RSSI);
	value &= ~EN_RSSI;
	
	SSV7241_Write_Reg(W_REG | RSSI, value); 	
}

void SSV7241_SetChannel(uint8 channel)
{
	SSV7241_Write_Reg(W_REG | RF_CH, channel); 
}

void SSV7241_Enable_dynamic_payload(uint8 dpl_px)
{
	uint8 value;
	
	SSV7241_Feature_DPL(1);
	value = SSV7241_Read_Reg(DYNPD);
	value |= dpl_px;
	SSV7241_Write_Reg(W_REG | DYNPD, value); 
}

void SSV7241_InitSPI()
{
	//========================= USER DEFINE ==================
	CS_High;
	GPIO_SetMode(IO_STATE_CE,GPIO_MODE_OUTPUT);
	CE_Low;
	//========================= USER DEFINE ==================
}

uint8 SPI0_communication(uint8 SPI_byte)
{
	//========================= USER DEFINE ==================
	SPI_WRITE_TX(SPI0, SPI_byte);
	SPI_ENABLE(SPI0);

	while(SPI_GetStatus(SPI0, SPI_BUSY_MASK));
	SPI_byte=SPI_READ_RX(SPI0);
	//========================= USER DEFINE ==================
	return (SPI_byte);
} // END SPI_Transfer

//**********************************************************
void	SSV7241_FHSS_TRIG(void)
{
		GPIO_SetMode(IO_STATE_INT,GPIO_MODE_INPUT);
		GPIO_EnableInt(PD, 3, GPIO_INT_FALLING);
		NVIC_EnableIRQ(EINT1_IRQn);
		channel=0;
		ChannelFlag=0xff;
		RxData.index_old=0xff;
		CE_Low;
		SSV7241_SetChannel(ChannelTable[channel]);
		CE_High;
		SSV7241_ClearRX_DR();
		SSV7241_FlushRX();
		RxState=0xff;
}


void SYS_Delay_Init(uint32_t us)
{
	SysTick->LOAD = us * CyclesPerUs;
	SysTick->VAL  = (0x00);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void SSV7241_Enable(void)
{
	/* Init ssv7241 */
	SSV7241_InitSPI();
	SSV7241_Init();
	SSV7241_PowerOn();
	DelayMsec(5);
	SSV7241_RX_Mode();
	SSV7241_EnableRSSI();
	SSV7241_FHSS_TRIG();
}
void match_address_process()
{
	uint8_t State = (1<<LED_MATCH_ADDRESS);
	
	if(RxData.BUF[0]==0x77) {
		MATCH_ADDRESS[0] = RxData.BUF[1];
		MATCH_ADDRESS[1] = RxData.BUF[2];
		MATCH_ADDRESS[2] = RxData.BUF[3];
		MATCH_ADDRESS[3] = RxData.BUF[4];
		MATCH_ADDRESS[4] = RxData.BUF[5];	
		SSV7241_TxACKPacket(&State,1);
		if (report_format == REPORT_FORMAT_TEXT)
		printf("RxData.BUF[0]:%x %x %x %x %x %x\n",RxData.BUF[0],MATCH_ADDRESS[0],MATCH_ADDRESS[1],
						MATCH_ADDRESS[2],MATCH_ADDRESS[3],MATCH_ADDRESS[4]);
	}
	else if(RxData.BUF[0]==0x88) {
		MATCH_ADDRESS_VALID = true;
		State+=1;
		SSV7241_TxACKPacket(&State,1);
		if (report_format == REPORT_FORMAT_TEXT)
		printf("RxData.BUF[0]:%x\n",RxData.BUF[0]);
	}
	
}
/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1(P5.2) default IRQ, declared in startup_Mini51.s.
 */
void EINT1_IRQHandler(void)
{
	uint8_t Rx_num,Databuf[32],i,State;
	State=GetLedState();

	GPIO_CLR_INT_FLAG(PD, BIT3);

	if(RxState==1) {
		SetTickSSVRC(4);
		Rx_num=SSV7241_RxPacket(Databuf);
		
		if(Rx_num) {
			RxData.index_old=RxData.index;
			ChannelFlag=Databuf[0];
			channel=Databuf[1];
			RxData.index=Databuf[2];
			NUM=Rx_num-3;
			
			for(i=0;i<NUM;i++)
				RxData.BUF[i]=Databuf[i+3];
			
			RxState=2;
			SSV7241_Write_Reg(W_REG | EN_AA,0x00);
			
			if(!MATCH_ADDRESS_PROCESS)
				MATCH_ADDRESS_VALID = true;
			else {
				if(MATCH_ADDRESS_VALID==true) {
					SSV7241_Write_Buf(W_REG | TX_ADDR,MATCH_ADDRESS,5);
					SSV7241_Write_Buf(W_REG | RX_ADDR_P0,MATCH_ADDRESS,5);	
					UpdateFlashRxAddress();
					MATCH_ADDRESS_PROCESS = false;
				}
			}
			
			NO_ACK_SEQUENCE_COUNT = 0;
			SSV_RC_CONNECT = true;
		}
	}
	else if(RxState==2) {
		SSV7241_Read_Buf(0x18,Databuf,2);			
		SSV7241_FlushRX();
		SSV7241_ClearRX_DR();
		RxData.num=NUM;
		RxState=3;
		SSV7241_Write_Reg(W_REG | EN_AA,0x01);
		RxData.rssi=0;
	
		if(Databuf[0]&0xe0) RxData.rssi|=0x01;
		if(Databuf[1]&0xe0) RxData.rssi|=0x02;
		
		RSSIf = RSSIf*0.98f + RxData.rssi*0.02f;
		
		if(MATCH_ADDRESS_PROCESS) {
			match_address_process();
		}
		else
			SSV7241_TxACKPacket(&State,1);
	}
	else if(RxState==0xff) {
		SetTickSSVRC(4);
		RxState=1;
		SSV7241_FlushRX();
		SSV7241_ClearRX_DR();
	}
}
int GetSSV_TickCount()
{
	return SSV_TICKCNT;
}
void SysTick_SSV(void)
{
	SetTickSSVRC(7);
	NO_ACK_SEQUENCE_COUNT++;
	SSV_TICKCNT++;
	if((MATCH_ADDRESS_VALID==true)&&(NO_ACK_SEQUENCE_COUNT>RC_LOOSE_COUNT))
		SSV_RC_CONNECT = false;
	
	if(RxState==1) {
		ChannelFlag&=~ChannelBit[channel];
		RxData.rssi=0;
		
		if(MATCH_ADDRESS_VALID==false) {
				if(NO_ACK_SEQUENCE_COUNT>MATCH_ADDRESS_INVALID_COUNT) {
					SetMatchAddress(RX_ADDRESS);
					SSV7241_Write_Buf(W_REG | TX_ADDR,MATCH_ADDRESS,5);
					SSV7241_Write_Buf(W_REG | RX_ADDR_P0,MATCH_ADDRESS,5);	
					MATCH_ADDRESS_PROCESS = true;	
					MATCH_ADDRESS_VALID = false;
					//SetTickSSVRC(1);
					//printf("MATCH_ADDRESS_PROCESS:%d\n",MATCH_ADDRESS_PROCESS);
				}
		}
				//printf("NO_ACK_SEQUENCE_COUNT:%d\n",NO_ACK_SEQUENCE_COUNT);
	}
	else if(RxState==2) {
		uint8_t i;
		
		i=RxData.rssi;
		SSV7241_TxACKPacket(&i,1);
		SSV7241_Write_Reg(W_REG | EN_AA,0x01);	
		RxData.num=NUM;
	}
	
	if(ChannelFlag==0) {
		channel=0;
		ChannelFlag=0xff;
		RxState=0xff;
		SSV7241_Write_Reg(W_REG | EN_AA,0x01); 
	}
	else {
		do {
			channel++;
			
			if(channel>=8) channel=0;
		} while((ChannelBit[channel]&ChannelFlag)==0);
		
		RxState=1;
	}
	
	CE_Low;
	SSV7241_SetChannel(ChannelTable[channel]);
	CE_High;
	SSV7241_ClearRX_DR();
	SSV7241_FlushRX();
}



