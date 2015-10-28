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
#ifndef _GPS_H
#define _GPS_H
#include "Def.h"
#define GPS_LEAD_FILTER                      // Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation
#define GPS_FILTERING                        // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
#define GPS_WP_RADIUS              200       // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
#define NAV_SLEW_RATE              30        // Adds a rate control to nav output, will smoothen out nav angle spikes
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 128
/* GPS navigation can control the heading */
#define NAV_CONTROLS_HEADING       true      // copter faces toward the navigation point, maghold must be enabled for it
#define NAV_TAIL_FIRST             false     // true - copter comes in with tail first 
#define NAV_SET_TAKEOFF_HEADING    true      // true - when copter arrives to home position it rotates it's head to takeoff direction
// **********************
// GPS Status Information
// **********************
typedef struct {
int16_t  GPS_angle[2];                      // the angles that must be applied for GPS correction
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
int16_t  GPS_directionToHome;                         // direction to home - unit: degree
uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
uint8_t  GPS_update;                              // a binary toogle to distinct a GPS position update
uint16_t GPS_ground_course;                       //                   - unit: degree*10
uint8_t  GPS_Present;                             // Checksum from Gps serial
uint8_t  GPS_Enable;
uint8_t  GPS_Fixed;
} GPS_Info_T;
typedef struct {
uint32_t  GPS_coord[2];
uint8_t  GPS_numSat;
uint8_t  GPS_Fixed;
} GPS_Report_T;
typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
} PID_PARAM;

#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
typedef struct {
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1;
} flags_gps_t;
// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees
#define LAT  0
#define LON  1

#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
typedef struct PID_ {
  float   integrator; // integrator value
  int32_t last_input; // last input for derivative
  float   lastderivative; // last derivative for low-pass filter
  float   output;
  float   derivative;
} PID;
#define _X 1
#define _Y 0

#define RADX100                    0.000174532925  
#define CROSSTRACK_GAIN            1
#define NAV_SPEED_MIN              100    // cm/sec
#define NAV_SPEED_MAX              300    // cm/sec
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)
void setupGPS(void);
void GPSCommandProcess(void);
GPS_Info_T* GetGPSInfo(void);
#endif
