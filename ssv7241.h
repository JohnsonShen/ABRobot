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
#ifndef __SSV7241_H__
#define __SSV7241_H__
#include "spi.h"
#include <stdbool.h>
/* SSV7241 Commands */
#define R_REG             0x00
#define W_REG             0x20
#define R_RX_PLOAD        0x61
#define W_TX_PLOAD        0xA0
#define FLUSH_TX          0xE1
#define FLUSH_RX          0xE2
#define REUSE_TX_PL       0xE3
#define R_RX_PL_WID       0x60
#define W_ACK_PLOAD       0xA8
#define W_TX_PLOAD_NOACK  0xB0
#define NOP               0xFF

/* SSV7241 Registers */
#define CFG_TOP           0x00
#define MASK_RX_DR        0x40
#define MASK_TX_DS        0x20
#define MASK_MAX_RT       0x10
#define EN_CRC            0x08
#define CRC_2B            0x04
#define PWR_ON            0x02
#define RX_ON             0x01

/* Enable/Disable Auto ACK */
#define EN_AA             0x01
#define ENAA_P5           0x20
#define ENAA_P4           0x10
#define ENAA_P3           0x08
#define ENAA_P2           0x04
#define ENAA_P1           0x02
#define ENAA_P0           0x01

/* Enable/Disable RX pipe number */
#define EN_RXADDR         0x02
#define ENRX_P5           0x20
#define ENRX_P4           0x10
#define ENRX_P3           0x08
#define ENRX_P2           0x04
#define ENRX_P1           0x02
#define ENRX_P0           0x01

/* Address Width setup */
#define SETUP_AW          0x03

/* Retransmit count and delay setting */
#define SETUP_RETR        0x04
#define ARD_250us         0x00
#define ARD_500us         0x10
#define ARD_750us         0x20
#define ARD_1000us        0x30
#define ARD_1250us        0x40
#define ARD_1500us        0x50
#define ARD_1750us        0x60
#define ARD_2000us        0x70
#define ARD_2250us        0x80
#define ARD_2500us        0x90
#define ARD_2750us        0xA0
#define ARD_3000us        0xB0
#define ARD_3250us        0xC0
#define ARD_3500us        0xD0
#define ARD_3750us        0xE0
#define ARD_4000us        0xF0
#define ARC_0             0x00
#define ARC_1             0x01
#define ARC_2             0x02
#define ARC_3             0x03
#define ARC_4             0x04
#define ARC_5             0x05
#define ARC_6             0x06
#define ARC_7             0x07
#define ARC_8             0x08
#define ARC_9             0x09
#define ARC_10            0x0A
#define ARC_11            0x0B
#define ARC_12            0x0C
#define ARC_13            0x0D
#define ARC_14            0x0E
#define ARC_15            0x0F

/* Channel number is from 0 (2400) to 127 (2527) */
#define RF_CH             0x05

/* TX Power and Rate setting */
#define SETUP_RF          0x06
#define RATE_1M           0x00
#define RATE_2M           0x08
#define RATE_250K         0x20
#define SSV_RF_DR_2M      RATE_2M
#define SSV_RF_DR_1M      RATE_1M
#define SSV_RF_DR_250k    RATE_250k
#define PWR_LEVEL1        0x00
#define PWR_LEVEL2        0x02
#define PWR_LEVEL3        0x04
#define PWR_LEVEL4        0x06

/* Interrupt status */
#define STATUS            0x07
#define RX_DR             0x40
#define TX_DS             0x20
#define MAX_RT            0x10
#define RX_P_NO_5         0x0C
#define RX_P_NO_4         0x0A
#define RX_P_NO_3         0x08
#define RX_P_NO_2         0x06
#define RX_P_NO_1         0x04
#define RX_P_NO_0         0x02

/* AA Retransmition Setting */
#define OBSERVE_TX        0x08
/* Value from 0 to 15, clear by changing channel */
#define PLOS_CNT          0xF0
/* Value from 0 to 15, clear by sending packet successfully */
#define ARC_CNT           0x0F

/* RSSI indicator and Enable/Disable Setting */
#define RSSI              0x09
#define EN_RSSI           0x10
#define RSSI_2            0x02 // - 59dBm
#define RSSI_1            0x01 // - 69dBm

/* TX and RX Address setting */
#define RX_ADDR_P0        0x0A
#define RX_ADDR_P1        0x0B
#define RX_ADDR_P2        0x0C
#define RX_ADDR_P3        0x0D
#define RX_ADDR_P4        0x0E
#define RX_ADDR_P5        0x0F
#define TX_ADDR           0x10

/* RX width, vaule from 1 to 32 */
#define RX_PW_P0          0x11
#define RX_PW_P1          0x12
#define RX_PW_P2          0x13
#define RX_PW_P3          0x14
#define RX_PW_P4          0x15
#define RX_PW_P5          0x16

/* FIFO Status */
#define STATUS_FIFO       0x17
#define TX_REUSE          0x40
#define TX_FULL           0x20
#define TX_EMPTY          0x10
#define RX_FULL           0x02
#define RX_EMPTY          0x01

/* RSSIREC */
#define RSSIREC           0x18

/* Enable/Disable Dynamic Paylod Per Pipe Setting */
#define DYNPD             0x1C
#define DPL_P5            0x20
#define DPL_P4            0x10
#define DPL_P3            0x08
#define DPL_P2            0x04
#define DPL_P1            0x02
#define DPL_P0            0x01

/* Enable/Disable Dynamic Payload , ACK Payload, Dynamic ACK function */
#define FEATURE           0x1D
#define EN_DPL            0x04
#define	EN_ACK_PAY        0x02
#define EN_DYN_ACK        0x01

#define BANK_SEL          0x1F
#ifdef M451
#define IO_STATE_CE       PD, BIT2
#define IO_CE             PD2
#define IO_STATE_INT      PD, BIT3
#define IO_INT            PD3
//========================   USER DEFINE ==============================
/* SSV7241 CSN, CE, IRQ setting */
#define CS_Low            SPI_SET_SS_LOW(SPI0)//SPI0->SSCTL|= SPI_SS// SPI->SSR |= SPI_SSR_SSR_Msk// CSN = 0
#define CS_High           SPI_SET_SS_HIGH(SPI0)//SPI0->SSCTL &= ~SPI_SS//SPI->SSR &= ~SPI_SSR_SSR_Msk// CSN = 1
#define CE_Low            IO_CE = 0 // CE = 0
#define CE_High           IO_CE = 1// CE = 1
#define IRQ_Status        // IRQ : Low Active
//#define SYS_Delay_10us() // waiting for 10us
#endif

#define TX_ADR_WIDTH    5   
#define RX_ADR_WIDTH    5 

//type define if necessary
#define uint8 uint8_t
#define uint32 uint32_t

typedef struct
{
	uint8_t  num;
	uint8_t	index_old;
	uint8_t	index;
	uint8_t	rssi;
	uint8_t  BUF[29];
} RF_DATA;

#define TxNUM 8
//========================   USER DEFINE ==============================

extern void SSV7241_Init(void);

/* register and data read/write */
extern void SSV7241_Write_Reg(uint8 reg,uint8 value);
extern uint8 SSV7241_Read_Reg(uint8 reg);
extern void SSV7241_Write_Buf(uint8 reg, uint8 *pBuf, uint8 len);
extern void SSV7241_Read_Buf(uint8 reg,uint8 *pBuf,uint8 len);

/* Interrupt function */
extern void SSV7241_ClearRX_DR(void);
extern void SSV7241_ClearTX_DS(void);
extern void SSV7241_ClearTX_DS(void);

/* TX/RX function */
extern uint8 SSV7241_RxPacket(uint8 *rxbuf);
extern void SSV7241_TxPacket(uint8 *txbuf, uint8 len);
void SSV7241_TxPacketWithoutAck(uint8 *txbuf, uint8 len);
void SSV7241_TxACKPacket(uint8 *txbuf, uint8 len);

extern uint8 SSV7241_CheckID(void);
extern void SSV7241_PowerOn(void);
extern void SSV7241_PowerOff(void);

/* Feature Settingg */
extern void SSV7241_Feature_DYN_ACK(uint8 Dyn_ack);
extern void SSV7241_Feature_DPL(uint8 Dpl);
extern void SSV7241_Feature_ACK_PAY(uint8 Ack_pay);

/* RSSI Setting */
extern void SSV7241_EnableRSSI(void);
extern void SSV7241_DisableRSSI(void);

/* Mode selection */
extern void SSV7241_TX_Mode(void);
extern void SSV7241_RX_Mode(void);

extern void SSV7241_SetChannel(uint8 channel);

extern uint8 SSV7241_isTXReady(void);
extern void SSV7241_Enable_dynamic_payload(uint8 dpl_px);
extern void SSV7241_FlushRX(void);
extern void SSV7241_FlushTX(void);

extern void SSV7241_InitSPI(void);
extern uint8 SPI0_communication(uint8 SPI_byte);

extern void SSV7241_Enable(void);
extern bool CheckLowRSSI(void);
extern uint8_t GetRSSI(void);
extern float GetRSSIf(void);
extern void GetMatchAddress(uint8_t* RX_ADDRESS);
extern int GetSSV_TickCount(void);
extern bool GetMatchAddressProcess(void);
#endif

