/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Adapted to Cortex-M4 Fly Controller by Nuvoton

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================*/
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
#include "AK8975.h"
#include "I2CDev.h"
#include "Timer_Ctrl.h"
static uint8_t devAddr;
static uint8_t buffer[6];
static int16_t hx, hy, hz;
/** Default constructor, uses default I2C address.
 * @see AK8975_DEFAULT_ADDRESS
 */
void AK8975_address() {
    devAddr = AK8975_DEFAULT_ADDRESS;
}
/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool AK8975_testConnection() {
    if (I2C_readByte(devAddr, AK8975_RA_WIA, buffer,1) == 1) {
        return (buffer[0] == 0x48);
    }
    return false;
}

uint8_t AK8975_getDeviceID() {
    I2C_readByte(devAddr, AK8975_RA_WIA, buffer,0);
    return buffer[0];
}

// INFO register

uint8_t AK8975_getInfo() {
    I2C_readByte(devAddr, AK8975_RA_INFO, buffer,0);
    return buffer[0];
}

// ST1 register

bool AK8975_getDataReady() {
    I2C_readBit(devAddr, AK8975_RA_ST1, AK8975_ST1_DRDY_BIT, buffer,0);
    return buffer[0];
}

// H* registers
void AK8975_getHeading(int16_t *x, int16_t *y, int16_t *z) {
	if(ChronographRead(ChronMAG)>= AK8975_READ_DELAY)
  {
		ChronographSet(ChronMAG);
		I2C_readBytes(devAddr, AK8975_RA_HXL, 6, buffer, 0);
		hx = (((int16_t)buffer[1]) << 8) | buffer[0];
		hy = (((int16_t)buffer[3]) << 8) | buffer[2];
    hz = (((int16_t)buffer[5]) << 8) | buffer[4];
		
    I2C_writeByte(devAddr, AK8975_RA_CNTL, AK8975_MODE_SINGLE);
    //DelayMsec(10);
    //while(!AK8975_getDataReady());
	}
	*x = hx;
  *y = hy;
  *z = hz;
    
}
int16_t AK8975_getHeadingX() {
    I2C_writeByte(devAddr, AK8975_RA_CNTL, AK8975_MODE_SINGLE);
    DelayMsec(8);
    I2C_readBytes(devAddr, AK8975_RA_HXL, 2, buffer,0);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}
int16_t AK8975_getHeadingY() {
    I2C_writeByte(devAddr, AK8975_RA_CNTL, AK8975_MODE_SINGLE);
    DelayMsec(8);
    I2C_readBytes(devAddr, AK8975_RA_HYL, 2, buffer,0);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}
int16_t AK8975_getHeadingZ() {
    I2C_writeByte(devAddr, AK8975_RA_CNTL, AK8975_MODE_SINGLE);
    DelayMsec(8);
    I2C_readBytes(devAddr, AK8975_RA_HZL, 2, buffer,0);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

// ST2 register
bool AK8975_getOverflowStatus() {
    I2C_readBit(devAddr, AK8975_RA_ST2, AK8975_ST2_HOFL_BIT, buffer,0);
    return buffer[0];
}
bool AK8975_getDataError() {
    I2C_readBit(devAddr, AK8975_RA_ST2, AK8975_ST2_DERR_BIT, buffer,0);
    return buffer[0];
}

// CNTL register
uint8_t AK8975_getMode() {
    I2C_readBits(devAddr, AK8975_RA_CNTL, AK8975_CNTL_MODE_BIT, AK8975_CNTL_MODE_LENGTH, buffer,0);
    return buffer[0];
}
void AK8975_setMode(uint8_t mode) {
    I2C_writeBits(devAddr, AK8975_RA_CNTL, AK8975_CNTL_MODE_BIT, AK8975_CNTL_MODE_LENGTH, mode);
}
void AK8975_reset() {
    I2C_writeBits(devAddr, AK8975_RA_CNTL, AK8975_CNTL_MODE_BIT, AK8975_CNTL_MODE_LENGTH, AK8975_MODE_POWERDOWN);
}

// ASTC register
void AK8975_setSelfTest(bool enabled) {
    I2C_writeBit(devAddr, AK8975_RA_ASTC, AK8975_ASTC_SELF_BIT, enabled);
}

// ASA* registers
void AK8975_getAdjustment(int8_t *x, int8_t *y, int8_t *z) {
    I2C_readBytes(devAddr, AK8975_RA_ASAX, 3, buffer,0);
    *x = buffer[0];
    *y = buffer[1];
    *z = buffer[2];
}
void AK8975_setAdjustment(int8_t x, int8_t y, int8_t z) {
    buffer[0] = x;
    buffer[1] = y;
    buffer[2] = z;
    I2C_writeBytes(devAddr, AK8975_RA_ASAX, 3, buffer);
}
uint8_t AK8975_getAdjustmentX() {
    I2C_readByte(devAddr, AK8975_RA_ASAX, buffer,0);
    return buffer[0];
}
void AK8975_setAdjustmentX(uint8_t x) {
    I2C_writeByte(devAddr, AK8975_RA_ASAX, x);
}
uint8_t AK8975_getAdjustmentY() {
    I2C_readByte(devAddr, AK8975_RA_ASAY, buffer,0);
    return buffer[0];
}
void AK8975_setAdjustmentY(uint8_t y) {
    I2C_writeByte(devAddr, AK8975_RA_ASAY, y);
}
uint8_t AK8975_getAdjustmentZ() {
    I2C_readByte(devAddr, AK8975_RA_ASAZ, buffer,0);
    return buffer[0];
}
void AK8975_setAdjustmentZ(uint8_t z) {
    I2C_writeByte(devAddr, AK8975_RA_ASAZ, z);
}
bool AK8975_initialize() {
	bool connect;
	AK8975_address();
	connect = AK8975_testConnection();
	if(connect)
		AK8975_reset();
	
	return connect;
}
