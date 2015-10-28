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
#include "Def.h"
#include "RC.h"
#include "AHRSLib.h"
#include "controller.h"
#include "motors.h"
#include "pid.h"
#include "math.h"
#include "sensors.h"
#include "AltHold.h"
#include "Timer_Ctrl.h"

#define ROLL_DEG_MAX  60
#define PITCH_DEG_MAX 60
#define YAW_DEG_MAX   80
#define THRUST_MAX    90
#define COS_30        0.866f
#define COS_60        0.5f
static float gyro[3]; // Gyro axis data in deg/s
static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;
static float vSpeed       = 0.0f; // Vertical speed (world frame) integrated from vertical acceleration
static float accWZ        = 0.0f;
static float headHold     = 0.0f;
static float headFreeHold = 0.0f;
static bool magMode = false;
static bool headFreeMode = false;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM[MOTOR_NUMBER];

char flip = 0;
void HoldHead(void)
{
	headHold = eulerYawActual;
}
void HoldHeadFree(void)
{
	headFreeHold = eulerYawActual;
}
bool getMagMode()
{
	return magMode;
}
bool getHeadFreeMode()
{
	return headFreeMode;
}
void stabilizerInit(void)
{
	controllerInit();
	motorsInit();
#if STACK_BARO
	SetAltHoldPIDObj(GetAltHoldPIDObj());
#endif
	rollRateDesired = 0;
	pitchRateDesired = 0;
	yawRateDesired = 0;
}
void DetectFlip()
{
	if(eulerRollActual>ROLL_DEG_MAX)
		flip = 1;
	else if(eulerRollActual<-ROLL_DEG_MAX)
		flip = 1;
	
	if(eulerPitchActual>PITCH_DEG_MAX)
		flip = 1;
	else if(eulerPitchActual<-PITCH_DEG_MAX)
		flip = 1;
}
float GetAccWZ()
{
	return accWZ;
}
char GetFlip()
{
	return flip;
}
void ClearFlip()
{
	flip = 0;
}

void commanderGetRPY()
{
	int16_t rcData[RC_CHANS], rc_roll, rc_pitch, rc_yaw, rc_aux2;
	
	getRC(rcData);
	rc_roll = (rcData[ROLL_CH] - RC_ROLL_MID);
	rc_pitch = (rcData[PITCH_CH] - RC_PITCH_MID);
	rc_yaw = (rcData[YAW_CH] - RC_YAW_MID);
	rc_aux2 = rcData[AUX2_CH];
	
	if(rc_aux2<RC_ONE_THIRD) {
		magMode = false;
		headFreeMode = false;
	} else if(rc_aux2>RC_TWO_THIRD) {
		if(!headFreeMode)
			HoldHeadFree();
		headFreeMode = true;
	}
	else {
		if(!magMode)
			HoldHead();
		
		magMode = true;
		headFreeMode = false;
	}
	
	if(headFreeMode) { //to optimize
		float radDiff = (headFreeHold - eulerYawActual) * 0.0174533f; // where PI/180 ~= 0.0174533
		float cosDiff = cos(radDiff);
		float sinDiff = sin(radDiff);
		int16_t rcCommand_PITCH = rc_pitch*cosDiff + rc_roll*sinDiff;
		
		rc_roll =  rc_roll*cosDiff - rc_pitch*sinDiff; 
		rc_pitch = rcCommand_PITCH;
		//if((GetFrameCount()%18)==0)
		//printf("YA:%f,HF:%f,Diff:%f\n",eulerYawActual,headFreeHold,radDiff);
	}

	if (fabs(rc_yaw)<RC_YAW_DEAD_BAND) {
		if(magMode&&(!headFreeMode)) {
			int16_t dif = eulerYawActual - headHold;
			
			if(dif<-180)
				dif = dif + 360;
			else if(dif>180)
				dif = dif - 360;
			
			eulerYawDesired = dif;
		}
		else 
			eulerYawDesired = 0;
	} 
	else {
		if(magMode)
			HoldHead();
		
		if(rc_yaw>0)
			eulerYawDesired = (rc_yaw*YAW_DEG_MAX/(RC_YAW_MAX-RC_YAW_MID));
		else
			eulerYawDesired = -(rc_yaw*YAW_DEG_MAX/(RC_YAW_MIN-RC_YAW_MID));
	}

	if(rc_roll>0)
		eulerRollDesired = (rc_roll*ROLL_DEG_MAX/(RC_ROLL_MAX-RC_ROLL_MID));
	else
		eulerRollDesired = -(rc_roll*ROLL_DEG_MAX/(RC_ROLL_MIN-RC_ROLL_MID));
	
	if(rc_pitch>0)
		eulerPitchDesired = (rc_pitch*PITCH_DEG_MAX/(RC_PITCH_MAX-RC_PITCH_MID));
	else
		eulerPitchDesired = -(rc_pitch*PITCH_DEG_MAX/(RC_PITCH_MIN-RC_PITCH_MID));
}
void commanderGetThrust()
{
	int16_t rcData[RC_CHANS], rc_thrust;
	char arm_min_thr = RC_THR_ARM - RC_THR_MIN;
	
	getRC(rcData);
	rc_thrust = GetRCThrust();
	if(checkArm()) {
		if(rc_thrust<arm_min_thr) {
#if STACK_BARO
			if(!GetAutoLandMode()) {
			rc_thrust = arm_min_thr;
			eulerYawDesired = 0;
			controllerResetAllPID();
		}
#else
			rc_thrust = arm_min_thr;
			eulerYawDesired = 0;
			controllerResetAllPID();
#endif
		}
	}
	else
		rc_thrust = 0;
	
	if(rc_thrust<=0)
		actuatorThrust = 0;
	else
		actuatorThrust = rc_thrust;
}
bool isArmMinThrottle()
{
	if((int)actuatorThrust<=(RC_THR_ARM - RC_THR_MIN))
		return true;
	else
		return false;
}
float GetactuatorThrust()
{
	return actuatorThrust;
}
	
uint16_t limitThrust(int32_t value)
{
	if(value > UINT16_MAX) {
		value = UINT16_MAX;
	}
	else if(value < 0) {
		value = 0;
	}

	return (uint16_t)value;
}

static void distributePower(uint16_t thrust, int16_t roll,
                            int16_t pitch, int16_t yaw)
{
#ifdef HEX6X 
	motorPowerM[0] = limitThrust(thrust - COS_60*roll - COS_30*pitch + yaw);//PIDMIX(-1/2,-7/8,+1); //FRONT_R
	motorPowerM[1] = limitThrust(thrust - roll - yaw);//PIDMIX(-1  ,+0  ,-1); //RIGHT
	motorPowerM[2] = limitThrust(thrust - COS_60*roll + COS_30*pitch + yaw);//PIDMIX(-1/2,+7/8,+1); //REAR_R
	motorPowerM[3] = limitThrust(thrust + COS_60*roll + COS_30*pitch - yaw);//PIDMIX(+1/2,+7/8,-1); //REAR_L
	motorPowerM[4] = limitThrust(thrust + roll + yaw);//PIDMIX(+1  ,+0  ,+1); //LEFT
	motorPowerM[5] = limitThrust(thrust + COS_60*roll - COS_30*pitch - yaw);//PIDMIX(+1/2,-7/8,-1); //FRONT_L
#else
	motorPowerM[0] = limitThrust(thrust - roll - pitch + yaw);
	motorPowerM[1] = limitThrust(thrust - roll + pitch - yaw);
	motorPowerM[2] =  limitThrust(thrust + roll + pitch + yaw);
	motorPowerM[3] =  limitThrust(thrust + roll - pitch - yaw);
#endif

	//printf("%d  %d  %d  %d\n", motorPowerM1>>6, motorPowerM2>>6, motorPowerM3>>6, motorPowerM4>>6);
#ifdef BLDC
	motorPowerM[0] = (motorPowerM[0]) + RC_THR_MIN;
	motorPowerM[1] = (motorPowerM[1]) + RC_THR_MIN;
	motorPowerM[2] = (motorPowerM[2]) + RC_THR_MIN;
	motorPowerM[3] = (motorPowerM[3]) + RC_THR_MIN;
#ifdef HEX6X 
	motorPowerM[4] = (motorPowerM[4]) + RC_THR_MIN;
	motorPowerM[5] = (motorPowerM[5]) + RC_THR_MIN;
#endif
#else
	motorPowerM[0] = (motorPowerM[0])*5+0;
	motorPowerM[1] = (motorPowerM[1])*5+0;
	motorPowerM[2] = (motorPowerM[2])*5+0;
	motorPowerM[3] = (motorPowerM[3])*5+0;
#endif
	//printf("%d  %d  %d  %d\n", motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4);
	motorsSetRatio(MOTOR_M1, motorPowerM[0]);
	motorsSetRatio(MOTOR_M2, motorPowerM[1]);
	motorsSetRatio(MOTOR_M3, motorPowerM[2]);
	motorsSetRatio(MOTOR_M4, motorPowerM[3]);
#ifdef HEX6X 
	motorsSetRatio(MOTOR_M5, motorPowerM[4]);
	motorsSetRatio(MOTOR_M6, motorPowerM[5]);
#endif
}

void GetMotorPower(uint16_t* MotorPower)
{
	MotorPower[0] = (uint16_t)motorPowerM[0];
	MotorPower[1] = (uint16_t)motorPowerM[1];
	MotorPower[2] = (uint16_t)motorPowerM[2];
	MotorPower[3] = (uint16_t)motorPowerM[3];
#ifdef HEX6X 
	MotorPower[4] = (uint16_t)motorPowerM[4];
	MotorPower[5] = (uint16_t)motorPowerM[5];
#endif
}
bool IsMotorSpin() 
{
	uint8_t i;
	for(i=0; i<MOTOR_NUMBER; i++) {
		if((uint16_t)motorPowerM[i]<MOTOR_START_SPIN)
			return false;
	}
	return true;
}
void GetvSpeed(float* speed)
{
	*speed = vSpeed;
}

void SetvSpeed(float speed)
{
	vSpeed = speed;
}

// Deadzone
float deadband(float value, const float threshold)
{
	if (fabs(value) < threshold) {
		value = 0;
	}
	else if (value > 0) {
		value -= threshold;
	}
	else if (value < 0) {
		value += threshold;
	}
	return value;
}

void stabilizer()
{
	float Euler[3],Ve[3];
#if STACK_BARO	
	bool altHold =  GetAltHoldMode();
#endif	
	nvtGetEulerRPY(Euler);
	eulerRollActual = Euler[0];
	eulerPitchActual = Euler[1];
	eulerYawActual = Euler[2];

	DetectFlip();
	
	commanderGetRPY();
	
	nvtGetVelocity(Ve);
	accWZ = Ve[2];
	
	if(checkArm())
		vSpeed=accWZ;
	else
		vSpeed = 0;
	
	//printf("%f  %f  %f \n", eulerRollActual, eulerPitchActual, eulerYawActual);
	controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
				eulerRollDesired, eulerPitchDesired, -eulerYawDesired,\
				&rollRateDesired, &pitchRateDesired, &yawRateDesired);
	
#if STACK_BARO
	if(GetSensorInitState()&SENSOR_BARO) {
		if(GetFrameCount()%(int)(1.0f/ALTHOLD_UPDATE_FREQ/getUpdateDT())==0) {
		stabilizerAltHoldUpdate(&actuatorThrust);
		}
		if(!altHold)
			commanderGetThrust();
	}
	else
#endif
	commanderGetThrust();
	//printf("%f  %f  %f \n", rollRateDesired, pitchRateDesired, yawRateDesired);
	yawRateDesired = -eulerYawDesired;
	nvtGetCalibratedGYRO(gyro);
	controllerCorrectRatePID(gyro[0], gyro[1], gyro[2],
				rollRateDesired, pitchRateDesired, yawRateDesired);
	controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);
#ifdef DGB	
	printf("actuatorThrust:%d",actuatorThrust);
#endif
	if(GetFrameCount()>(MOTORS_ESC_DELAY*2)) {
	if (actuatorThrust > 0)
			distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
	else {
		distributePower(0, 0, 0, 0);
	}
		//printf("Th,Roll,Pitch,Yaw:%d,%d,%d,%d  ",actuatorThrust,actuatorRoll, actuatorPitch, -actuatorYaw);
}
}
