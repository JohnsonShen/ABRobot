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
#include "I2Cdev.h"
#include "Timer_Ctrl.h"
#include "Sensors.h"
#include "stdio.h"

#ifdef BMP280
#include "BMP280.h"
#include "math.h"
#define P0 1013.25f
#define CONVERSION_TIME_MS   15

uint8_t  dig_T2_[2], dig_T3_[2], dig_P2_[2], dig_P3_[2], dig_P4_[2],
         dig_P5_[2], dig_P6_[2], dig_P7_[2], dig_P8_[2], dig_P9_[2],
         dig_P1_[2], dig_T1_[2];

int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4,
         dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint16_t dig_P1, dig_T1;

float BMP280_T, BMP280_P, BMP280_uT, BMP280_uP, BMP280_A;
unsigned char BMP280_delay;
short oversampling = 2;
long signed int t_fine;
static uint32_t lastConv=0;

bool Int_BMP280(void)
{
	uint8_t ret;
	
	I2C_readBytes(BMP280_ADDRESS, BMP280_CHIP_ID_REG, 6, &ret, 1);
	if (ret != 0x58)
		return false;
	
	/* read calibration data */
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_T1_LSB_REG, 2, dig_T1_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_T2_LSB_REG, 2, dig_T2_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_T3_LSB_REG, 2, dig_T3_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P1_LSB_REG, 2, dig_P1_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P2_LSB_REG, 2, dig_P2_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P3_LSB_REG, 2, dig_P3_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P4_LSB_REG, 2, dig_P4_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P5_LSB_REG, 2, dig_P5_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P6_LSB_REG, 2, dig_P6_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P7_LSB_REG, 2, dig_P7_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P8_LSB_REG, 2, dig_P8_, 1);
	I2C_readBytes(BMP280_ADDRESS, BMP280_DIG_P9_LSB_REG, 2, dig_P9_, 1);
	
	dig_T1 = (((unsigned int)dig_T1_[1]<<8)|(unsigned int)dig_T1_[0]);
	dig_T2 = (((int)dig_T2_[1]<<8)|(int)dig_T2_[0]);
	dig_T3 = (((int)dig_T3_[1]<<8)|(int)dig_T3_[0]);
	dig_P1 = (((unsigned int)dig_P1_[1]<<8)|(unsigned int)dig_P1_[0]);
	dig_P2 = (((int)dig_P2_[1]<<8)|(int)dig_P2_[0]);
	dig_P3 = (((int)dig_P3_[1]<<8)|(int)dig_P3_[0]);
	dig_P4 = (((int)dig_P4_[1]<<8)|(int)dig_P4_[0]);
	dig_P5 = (((int)dig_P5_[1]<<8)|(int)dig_P5_[0]);
	dig_P6 = (((int)dig_P6_[1]<<8)|(int)dig_P6_[0]);
	dig_P7 = (((int)dig_P7_[1]<<8)|(int)dig_P7_[0]);
	dig_P8 = (((int)dig_P8_[1]<<8)|(int)dig_P8_[0]);
	dig_P9 = (((int)dig_P9_[1]<<8)|(int)dig_P9_[0]);
	return true;
}

char calcPressure()
{
	float var1 , var2, p ;
	
	var1 = ((float)t_fine/2.0f) - 64000.0f;
	var2 = var1 * (var1 * ((float)dig_P6)/32768.0f);	//not overflow
	var2 = var2 + (var1 * ((float)dig_P5)*2.0f);	//overflow	
	var2 = (var2/4.0f)+(((float)dig_P4)*65536.0f);	
	var1 = (((float)dig_P3) * var1 * var1/524288.0f + ((float)dig_P2) * var1) / 524288.0f;
	var1 = ((32768.0f + var1)/32768.0f)*((float)dig_P1);
		
	p = 1048576.0f- (float)BMP280_uP;		
	p = (p-(var2/4096.0f))*6250.0f/var1 ;	//overflow
		
	var1 = ((float)dig_P9)*p*p/2147483648.0f;	//overflow	
	var2 = p*((float)dig_P8)/32768.0f;
	p = p + (var1+var2+((float)dig_P7))/16.0f;
		
	BMP280_P = p/100.0f ;
	
	if(BMP280_P>1200.0f || BMP280_P < 800.0f)return (0);
	return (1);
}

char calcTemperature()
{
	float adc_T = BMP280_uT ;
		
	float var1 = (((float)adc_T)/16384.0f-((float)dig_T1)/1024.0f)*((float)dig_T2);
	float var2 = ((((float)adc_T)/131072.0f - ((float)dig_T1)/8192.0f)*(((float)adc_T)/131072.0f - ((float)dig_T1)/8192.0f))*((float)dig_T3);
	t_fine = (long signed int)(var1+var2);
		
	BMP280_T = (var1+var2)/5120.0f;
	
	if(BMP280_T>100.0f || BMP280_T <-100.0f)return 0;
	
	return (1);
}

char getUnPT()
{
	uint8_t Gdata[6];
	float factor;
	
	I2C_readBytes(BMP280_ADDRESS, BMP280_PRESSURE_MSB_REG,     6, &Gdata[0], 1);		//0xF7~0xFC
	/*
	I2C_readBytes(BMP280_ADDRESS, BMP280_PRESSURE_LSB_REG,     1, &Gdata[1], 1);		//0xF8
	I2C_readBytes(BMP280_ADDRESS, BMP280_PRESSURE_XLSB_REG,    1, &Gdata[2], 1);		//0xF9
	I2C_readBytes(BMP280_ADDRESS, BMP280_TEMPERATURE_MSB_REG,  1, &Gdata[3], 1);		//0xFA
	I2C_readBytes(BMP280_ADDRESS, BMP280_TEMPERATURE_LSB_REG,  1, &Gdata[4], 1);		//0xFB
	I2C_readBytes(BMP280_ADDRESS, BMP280_TEMPERATURE_XLSB_REG, 1, &Gdata[5], 1);		//0xFC*/

	factor = pow(2, 4);
	BMP280_uP = (( (Gdata[0] *256.0f) + Gdata[1] + (Gdata[2]/256.0f))) * factor ;	//20bit UP
	BMP280_uT = (( (Gdata[3] *256.0f) + Gdata[4] + (Gdata[5]/256.0f))) * factor ;	//20bit UT

	return(1);
}

char getTemperatureAndPressure()
{
	char result = getUnPT();
	
	if(result!=0){
		// calculate the temperature
		result = calcTemperature();
		if(result){
			// calculate the pressure
			result = calcPressure();
			if(result)
				return (1);
			else{									// pressure error ;
				//printf("pressure error");
				return (0);
					}
		}else { 									// temperature error ;
			//printf("temperature error");
				return (0);
					}
	}
	return (0);
}

char startMeasurment(void)
{
	unsigned char data[2], result;
	
	data[0] = BMP280_CTRL_MEAS_REG;
	switch (oversampling)
	{
		case 0:
			data[1] = BMP280_COMMAND_PRESSURE0;     
			BMP280_delay = 8;			
		break;
		case 1:
			data[1] = BMP280_COMMAND_PRESSURE1;     
			BMP280_delay = 10;			
		break;
		case 2:
			data[1] = BMP280_COMMAND_PRESSURE2;		
			BMP280_delay = 15;
		break;
		case 3:
			data[1] = BMP280_COMMAND_PRESSURE3;
			BMP280_delay = 24;
		break;
		case 4:
			data[1] = BMP280_COMMAND_PRESSURE4;
			BMP280_delay = 45;
		break;
		default:
			data[1] = BMP280_COMMAND_PRESSURE0;
			BMP280_delay = 9;
		break;
	}
	result = I2C_writeBytes(BMP280_ADDRESS, BMP280_CTRL_MEAS_REG, 1, &data[1]);
	
	if (result)// good write?
		return(BMP280_delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0);// or return 0 if there was a problem communicating with the BMP
}

float sealevel()
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(BMP280_P/pow(1-(BMP280_A/44330.0f),5.255f));
}

float BMP280_altitude()
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0f*(1-pow(BMP280_P/P0,1/5.255f)));
}

bool BMP280SelfTest(void)
{
  getTemperatureAndPressure();
  printf("Ground Altitude:%f\n", BMP280_altitude());
  return 1;
}

bool BMP280_GetData(float* pressure, float* temperature, float* asl)
{	
  char result;

  uint32_t now = getTickCount();
  if ((now - lastConv) < CONVERSION_TIME_MS){
     return false;
  }
  lastConv = now;
  result = startMeasurment();

  if(result!=0){
    result = getTemperatureAndPressure();
    *pressure = BMP280_P;
    *temperature = BMP280_T;
  }
  else {
		//printf("Error.");
		return false;
	}
	return true;
}

#endif
