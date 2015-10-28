/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
	Adapted to Cortex-M4 Fly Controller by Nuvoton
 ***************************************************************************/
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
#include <math.h>
#include "Adafruit_BMP085.h"
#include "I2Cdev.h"
#include "Timer_Ctrl.h"
#include "Sensors.h"
#ifdef BMP085
static uint32_t RawPressure;
static uint16_t RawTemperature;
uint8_t oversampling;
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
char ReadPressureDelay[4] = {5, 8, 14, 30};
float temperature;
bool begin(uint8_t mode) {
	if (mode > BMP085_ULTRAHIGHRES) 
		mode = BMP085_ULTRAHIGHRES;
	oversampling = mode;

	if (read8(0xD0,1) != 0x55) return false;

	/* read calibration data */
	ac1 = read16(BMP085_CAL_AC1);
	ac2 = read16(BMP085_CAL_AC2);
	ac3 = read16(BMP085_CAL_AC3);
	ac4 = read16(BMP085_CAL_AC4);
	ac5 = read16(BMP085_CAL_AC5);
	ac6 = read16(BMP085_CAL_AC6);

	b1 = read16(BMP085_CAL_B1);
	b2 = read16(BMP085_CAL_B2);

	mb = read16(BMP085_CAL_MB);
	mc = read16(BMP085_CAL_MC);
	md = read16(BMP085_CAL_MD);
	return true;
}

int32_t computeB5(int32_t UT) {
	int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
	int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
	return X1 + X2;
}
void TriggerRawTemperature(void) {
	write8(BMP085_CONTROL, BMP085_READTEMPCMD);
}
uint16_t readRawTemperature(void) {
	RawTemperature = read16(BMP085_TEMPDATA);
	return RawTemperature;
	}
void TriggerRawPressure(void) {
	write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
}
uint32_t readRawPressure(void) {
	RawPressure = read16(BMP085_PRESSUREDATA);
	RawPressure <<= 8;
	RawPressure |= read8(BMP085_PRESSUREDATA+2,0);
	RawPressure >>= (8 - oversampling);
	//printf(" RP:%d ",RawPressure);
	return RawPressure;
}
extern Sensor_T Sensor;
int32_t readPressure(void) {
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
	int32_t base;
	uint16_t raw[2];

	base = GetBaroBasePressure();
	nvtGetSensorRawBARO(raw);
	UT = raw[1];
	UP=base + (int16_t)raw[0];

#if BMP085_DEBUG == 1
	printf("UT = %d\n",UT);
	printf("UP = %d\n",UP);
#endif

	B5 = computeB5(UT);
	temperature = (float)((B5+8)/16)/10;
	//printf(" BMP:%f  ",temperature);
	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
	X1 = ((int32_t)ac3 * B6) >> 13;
	X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + (int32_t)3791)>>4);

	return p;
}

int32_t readSealevelPressure(float altitude_meters) {
	float pressure = readPressure();
	return (int32_t)(pressure / pow(1.0f-altitude_meters/44330, 5.255f));
}

float readTemperature(int32_t UT) {
	int32_t B5;     // following ds convention
	float temp;

  //UT = readRawTemperature();

#if BMP085_DEBUG == 2
	// use datasheet numbers!
	UT = 27898;
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
#endif

	B5 = computeB5(UT);
	temp = (B5+8) >> 4;
	temp /= 10;

	return temp;
}

float readAltitude(float sealevelPressure, float pressure) {
	float altitude;

	altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

	return altitude;
}


/*********************************************************************/

uint8_t read8(uint8_t a, uint16_t timeout) {
	uint8_t ret;
#if 0
	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
	Wire.write(a); // sends register address to read from
#else
	Wire.send(a); // sends register address to read from
#endif
	Wire.endTransmission(); // end transmission

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
	Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
#if (ARDUINO >= 100)
	ret = Wire.read(); // receive DATA
#else
	ret = Wire.receive(); // receive DATA
#endif
	Wire.endTransmission(); // end transmission
#endif
	I2C_readByte(BMP085_I2CADDR, a, &ret,timeout);
	return ret;
}

uint16_t read16(uint8_t a) {
	uint16_t ret;
	uint8_t buffer[2];
#if 0
	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
	Wire.write(a); // sends register address to read from
#else
	Wire.send(a); // sends register address to read from
#endif
	Wire.endTransmission(); // end transmission

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
	Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
#if (ARDUINO >= 100)
	ret = Wire.read(); // receive DATA
	ret <<= 8;
	ret |= Wire.read(); // receive DATA
#else
	ret = Wire.receive(); // receive DATA
	ret <<= 8;
	ret |= Wire.receive(); // receive DATA
#endif
	Wire.endTransmission(); // end transmission
#endif
	I2C_readBytes(BMP085_I2CADDR, a, 2, buffer,0);
	ret = buffer[0]<<8 | buffer[1];
	return ret;
}

void write8(uint8_t a, uint8_t d) {
#if 0
	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
	Wire.write(a); // sends register address to read from
	Wire.write(d);  // write data
#else
	Wire.send(a); // sends register address to read from
	Wire.send(d);  // write data
#endif
	Wire.endTransmission(); // end transmission
#endif
	I2C_writeByte(BMP085_I2CADDR, a, d);
}
#endif
