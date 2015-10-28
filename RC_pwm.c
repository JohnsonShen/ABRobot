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
#include "Def.h"
#include "retarget.h"
#include "Timer_Ctrl.h"
#include "RC.h"
#ifdef M451
#define RX_PORTB_COUNT   6
#define IO_RC_D2  PB, BIT0
#define IO_RC_D4  PB, BIT1
#define IO_RC_D5  PB, BIT6
#define IO_RC_D6  PB, BIT3
#define IO_RC_D7  PB, BIT4
#define IO_RC_D8  PB, BIT5
#else
#define RX_PORTC_COUNT   2
#define RX_PORTB_COUNT   4
#define IO_RC_D2	E_GPC, 4
#define IO_RC_D4	E_GPC, 5
#define IO_RC_D5	E_GPB, 3
#define IO_RC_D6	E_GPB, 2
#define IO_RC_D7	E_GPB, 1
#define IO_RC_D8	E_GPB, 0
#endif

#ifdef M451
#define RX_PORTB_BITS    (1<<0),(1<<1),(1<<6),(1<<3),(1<<4),(1<<5)
#else
#define RX_PORTC_BITS    (1<<4),(1<<5)
#define RX_PORTB_BITS    (1<<0),(1<<1),(1<<2),(1<<3)
#define RX_PIN_PORTB      E_GPB
#define RX_PIN_PORTC      E_GPC
static uint8_t RX_PORTC_PINS[RX_PORTC_COUNT] = {RX_PORTC_BITS};
#endif
uint16_t* rcValuePwm;
static uint8_t rcChannelPWM[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN};
static uint8_t RX_PORTB_PINS[RX_PORTB_COUNT] = {RX_PORTB_BITS};  

#define RX_PIN_CHECK(pin_pos, rc_value_pos)                                                        \
    if (mask & RX_PORT_PINS[pin_pos]) {                                                             \
      if (!(pin & RX_PORT_PINS[pin_pos])) {                                                         \
        dTime = cTime-edgeTime[pin_pos]; if (900<dTime && dTime<2200) rcValuePwm[rc_value_pos] = dTime; \
      } else edgeTime[pin_pos] = cTime;                                                              \
    }
#ifdef M451
void GPB_IRQHandler(void)
{
	int32_t mask;
	int32_t pin;
	uint8_t *RX_PORT_PINS = RX_PORTB_PINS;
	uint32_t cTime,dTime;
	static uint32_t edgeTime[8];
	
	mask = GPIO_GET_INT_FLAG(PB, BIT0|BIT1|BIT6|BIT3|BIT4|BIT5);

	if(mask) {
		GPIO_CLR_INT_FLAG(PB, mask);
	} 
	else {
		PB->INTSRC = PB->INTSRC;
		//printf("Un-expected interrupts.\n");
	}

	pin = PB0|(PB1<<1)|(PB6<<6)|(PB3<<3)|(PB4<<4)|(PB5<<5);

	cTime = micros();   
	
	RX_PIN_CHECK(0,AUX2PIN);
	RX_PIN_CHECK(1,AUX1PIN);
	RX_PIN_CHECK(2,YAWPIN);
	RX_PIN_CHECK(3,PITCHPIN);
	RX_PIN_CHECK(4,ROLLPIN);
	RX_PIN_CHECK(5,THROTTLEPIN);
	//printf("cTime:%d,dTime:%d\n",cTime,dTime);
}
#else
void GPABCallback(uint32_t u32GpaStatus, uint32_t u32GpbStatus)
{
	int32_t mask;
	int32_t pin;
	uint8_t *RX_PORT_PINS = RX_PORTB_PINS;
	uint32_t cTime,dTime;
	static uint32_t edgeTime[8];
	
	pin = DrvGPIO_GetPortBits(RX_PIN_PORTB);
	mask = u32GpbStatus;
	cTime = micros();                         
    
	RX_PIN_CHECK(0,0);
	RX_PIN_CHECK(1,1);
	RX_PIN_CHECK(2,2);
	RX_PIN_CHECK(3,3);
}
void GPCDCallback(uint32_t u32GpcStatus, uint32_t u32GpdStatus, uint32_t u32GpeStatus)
{
	int32_t mask;
	int32_t pin;
	uint8_t *RX_PORT_PINS = RX_PORTC_PINS;
	uint32_t cTime,dTime;
	static uint32_t edgeTime[8];
	
	pin = DrvGPIO_GetPortBits(RX_PIN_PORTC);
	mask = u32GpcStatus;
	cTime = micros();                         
    
	RX_PIN_CHECK(0,4);
	RX_PIN_CHECK(1,5);
}
#endif
void RC_PWM_Enable(char enable)
{
	if(enable) {
#ifdef M451
		GPIO_EnableInt(PB, 4, GPIO_INT_BOTH_EDGE);
		GPIO_EnableInt(PB, 5, GPIO_INT_BOTH_EDGE);
		GPIO_EnableInt(PB, 3, GPIO_INT_BOTH_EDGE);
		GPIO_EnableInt(PB, 6, GPIO_INT_BOTH_EDGE);
		GPIO_EnableInt(PB, 1, GPIO_INT_BOTH_EDGE);
		GPIO_EnableInt(PB, 0, GPIO_INT_BOTH_EDGE);
		NVIC_EnableIRQ(GPB_IRQn);
#else
		DrvGPIO_EnableInt(IO_RC_D2, E_IO_BOTH_EDGE, E_MODE_EDGE);
		DrvGPIO_EnableInt(IO_RC_D4, E_IO_BOTH_EDGE, E_MODE_EDGE);
		DrvGPIO_EnableInt(IO_RC_D5, E_IO_BOTH_EDGE, E_MODE_EDGE);
		DrvGPIO_EnableInt(IO_RC_D6, E_IO_BOTH_EDGE, E_MODE_EDGE);
		DrvGPIO_EnableInt(IO_RC_D7, E_IO_BOTH_EDGE, E_MODE_EDGE);
		DrvGPIO_EnableInt(IO_RC_D8, E_IO_BOTH_EDGE, E_MODE_EDGE);
#endif
	}
	else {
#ifdef M451
#else
		DrvGPIO_DisableInt(IO_RC_D2);
		DrvGPIO_DisableInt(IO_RC_D4);
		DrvGPIO_DisableInt(IO_RC_D5);
		DrvGPIO_DisableInt(IO_RC_D6);
		DrvGPIO_DisableInt(IO_RC_D7);
		DrvGPIO_DisableInt(IO_RC_D8);
#endif
	}
}
void RC_PWM_Init(void)
{
	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO);
	SYS->GPD_MFPL &= ~SYS_GPD_MFPL_PD3MFP_Msk;	
	SYS->GPD_MFPL |= SYS_GPD_MFPL_PD3MFP_GPIO;
#ifdef M451
	GPIO_SetMode(IO_RC_D2, GPIO_MODE_QUASI);
	GPIO_SetMode(IO_RC_D4, GPIO_MODE_QUASI);
	GPIO_SetMode(IO_RC_D5, GPIO_MODE_QUASI);
	GPIO_SetMode(IO_RC_D6, GPIO_MODE_QUASI);
	GPIO_SetMode(IO_RC_D7, GPIO_MODE_QUASI);
	GPIO_SetMode(IO_RC_D8, GPIO_MODE_QUASI);
#else
	DrvGPIO_Open(IO_RC_D2, E_IO_QUASI);
	DrvGPIO_Open(IO_RC_D4, E_IO_QUASI);
	DrvGPIO_Open(IO_RC_D5, E_IO_QUASI);
	DrvGPIO_Open(IO_RC_D6, E_IO_QUASI);
	DrvGPIO_Open(IO_RC_D7, E_IO_QUASI);
	DrvGPIO_Open(IO_RC_D8, E_IO_QUASI);
	DrvGPIO_SetIntCallback(GPABCallback, GPCDCallback);
#endif	
	rcValuePwm = getValue();
}
uint16_t readPwmRC(uint8_t chan) 
{
	uint16_t data;

	data = rcValuePwm[rcChannelPWM[chan]];

	return data;
}
bool IsPWMRCConnected()
{
	if(rcValuePwm[0]<RC_MIN_CMD)
		return false;
	else 
	return true;
}
int16_t GetPWMThrust()
{
	return ((readPwmRC(THR_CH) - RC_THR_MIN) + (RC_THR_HOV - RC_THR_MID));
}
int16_t GetPWMAltitude()
{
	return (readPwmRC(THR_CH) - RC_THR_MID);
}
