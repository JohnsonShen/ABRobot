// I2Cdev library collection - HMC5883L I2C device class
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// Adapted to Cortex-M4 Fly Controller by Nuvoton
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
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
#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdint.h>
#include "Def.h"
#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

#define HMC5883L_XY_RATIO_FACTOR   	1.16f
#define HMC5883L_Z_RATIO_FACTOR   	1.08f
#if 0
#define HMC5883L_ST_GAIN            HMC5883L_GAIN_440  // Gain value during self-test
#define HMC5883L_ST_GAIN_NBR        440
#else
#define HMC5883L_ST_GAIN            HMC5883L_GAIN_660  // Gain value during self-test
#define HMC5883L_ST_GAIN_NBR        660
#endif
#define HMC5883L_ST_ERROR           0.2f                // Max error
#define HMC5883L_ST_DELAY_MS        250                // delay in millisec during self test */
#define HMC5883L_ST_X_NORM          (int32_t)(HMC5883L_XY_RATIO_FACTOR * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_X_MIN           (int32_t)(HMC5883L_ST_X_NORM - (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_X_MAX           (int32_t)(HMC5883L_ST_X_NORM + (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_NORM          (int32_t)(HMC5883L_XY_RATIO_FACTOR * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Y_MIN           (int32_t)(HMC5883L_ST_Y_NORM - (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_MAX           (int32_t)(HMC5883L_ST_Y_NORM + (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_NORM          (int32_t)(HMC5883L_Z_RATIO_FACTOR * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Z_MIN           (int32_t)(HMC5883L_ST_Z_NORM - (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_MAX           (int32_t)(HMC5883L_ST_Z_NORM + (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))

bool hmc5883lInit(void);
bool hmc5883lTestConnection(void);
bool hmc5883lSelfTest(void);
bool hmc5883lEvaluateSelfTest(int16_t min, int16_t max, int16_t value, char* string);

// CONFIG_A register
uint8_t hmc5883lGetSampleAveraging(void);
void hmc5883lSetSampleAveraging(uint8_t averaging);
uint8_t hmc5883lGetDataRate(void);
void hmc5883lSetDataRate(uint8_t rate);
uint8_t hmc5883lGetMeasurementBias(void);
void hmc5883lSetMeasurementBias(uint8_t bias);

// CONFIG_B register
uint8_t hmc5883lGetGain(void);
void hmc5883lSetGain(uint8_t gain);

// MODE register
uint8_t hmc5883lGetMode(void);
void hmc5883lSetMode(uint8_t mode);

// DATA* registers
void hmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t hmc5883lGetHeadingX(void);
int16_t hmc5883lGetHeadingY(void);
int16_t hmc5883lGetHeadingZ(void);

// STATUS register
bool hmc5883lGetLockStatus(void);
bool hmc5883lGetReadyStatus(void);

// ID_* registers
uint8_t hmc5883lGetIDA(void);
uint8_t hmc5883lGetIDB(void);
uint8_t hmc5883lGetIDC(void);

// Others
void hmc5883lGetRatioFactor(float *x, float *y, float *z);

#endif /* HMC5883L_H_ */
