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
#include <stdio.h>
#include <math.h>
#include "M451Series.h"
#include "retarget.h"
#include "Timer_Ctrl.h"
#include "RC.h"
#include "gps.h"
#include "PID.h"
#define RXBUFSIZE   1024
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
#define DIGIT_TO_VAL(_x)        (_x - '0')
volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
#define FRAME_GGA  1
#define FRAME_RMC  2
#define GPS_LAG 0.5f                          //UBLOX GPS has a smaller lag than MTK and other
static int32_t  GPS_coord_lead[2];              // Lead filtered gps coordinates
int16_t  magHold; // [-180;+180]
int16_t last_velocityX = 0;
int16_t last_velocityY = 0;
extern uint8_t nav_mode; // Navigation mode
extern int16_t  nav[2];
extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

flags_gps_t fgps;

GPS_Info_T GPS_Info;
PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t LeadFilterX_get_position(int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - last_velocityX) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    last_velocityX = vel;

    return pos + vel_contribution + accel_contribution;
}
int32_t LeadFilterY_get_position(int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - last_velocityY) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    last_velocityY = vel;

    return pos + vel_contribution + accel_contribution;
}

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
  pid->integrator += ((float)error * pid_param->kI) * *dt;
  pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
  return pid->integrator;
}
    
int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
  float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  pid->derivative = (input - pid->last_input) / *dt;
  /// Low pass filter cut frequency for derivative calculation.
    
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
  // update state
  pid->last_input = input;
  pid->lastderivative    = pid->derivative;
  // add in derivative component
  return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderivative = 0;
}
static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static uint16_t GPS_wp_radius    = GPS_WP_RADIUS;
static int16_t actual_speed[2] = {0,0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles
 // The difference between the desired rate of travel and the actual rate of travel
  // updated after GPS read - 5-10hz
  static int16_t rate_error[2];
  static int32_t error[2];

  //Currently used WP
  static int32_t GPS_WP[2];

  ////////////////////////////////////////////////////////////////////////////////
  // Location & Navigation
  ////////////////////////////////////////////////////////////////////////////////
  // This is the angle from the copter to the "next_WP" location in degrees * 100
  static int32_t target_bearing;
  ////////////////////////////////////////////////////////////////////////////////
  // Crosstrack
  ////////////////////////////////////////////////////////////////////////////////
  // deg * 100, The original angle to the next_WP when the next_WP was set
  // Also used to check when we pass a WP
  static int32_t original_target_bearing;
  // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
  static int16_t crosstrack_error;
  ////////////////////////////////////////////////////////////////////////////////
  // The location of the copter in relation to home, updated every GPS read (1deg - 100)
  // static int32_t home_to_copter_bearing; /* unused */
  // distance between plane and home in cm
  // static int32_t home_distance; /* unused */
  // distance between plane and next_WP in cm
  static uint32_t wp_distance;
  
  // used for slow speed wind up when start navigation;
  static uint16_t waypoint_speed_gov;

  ////////////////////////////////////////////////////////////////////////////////////
  // moving average filter variables
  //
  
  #define GPS_FILTER_VECTOR_LENGTH 5
  
  static uint8_t GPS_filter_index = 0;
  static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
  static int32_t GPS_filter_sum[2];
  static int32_t GPS_data[2];
  static int32_t GPS_filtered[2];
  static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
  static uint16_t fraction3[2];
  
  // This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;
// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

uint8_t alarmArray[16];           // array
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  } ubx_header;
typedef struct {
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
  } ubx_nav_posllh;
typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
  } ubx_nav_solution;
typedef struct {
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
  } ubx_nav_velned;
  
  enum ubs_protocol_bytes {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
  };
  enum ubs_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
  };
  enum ubx_nav_status_bits {
    NAV_STATUS_FIX_VALID = 1
  };
  
  // Packet checksum accumulators
  static uint8_t _ck_a;
  static uint8_t _ck_b;
  
  // State machine state
  static uint8_t _step;
  static uint8_t _msg_id;
  static uint16_t _payload_length;
  static uint16_t _payload_counter;
  
//  static bool next_fix;
  //static uint8_t _class;

  //static uint8_t _disable_counter;
  static uint8_t _fix_ok;
  
  // Receive buffer
  static union {
    ubx_nav_posllh posllh;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[512];
   } _buffer;
  
char UBLOX_INIT[] = {
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
     0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
};
char UBLOX_115200[] = {"$PUBX,41,1,0003,0001,115200,0*1E\r\n"};
char UBLOX_57600[] = {"$PUBX,41,1,0003,0001,57600,0*2D\r\n"};
char UBLOX_38400[] = {"$PUBX,41,1,0003,0001,38400,0*26\r\n"};
char UBLOX_19200[] = {"$PUBX,41,1,0003,0001,19200,0*23\r\n"};

 bool UBLOX_parse_gps(void) {
    switch (_msg_id) {
    case MSG_POSLLH:
      //i2c_dataset.time                = _buffer.posllh.time;
      if(_fix_ok) {
        GPS_Info.GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_Info.GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_Info.GPS_altitude   = _buffer.posllh.altitude_msl / 1000;      //alt in m
      }
      fgps.GPS_FIX = _fix_ok;
      return true;        // POSLLH message received, allow blink GUI icon and LED
    case MSG_SOL:
      _fix_ok = 0;
      if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) _fix_ok = 1;
      GPS_Info.GPS_numSat = _buffer.solution.satellites;
      break;
    case MSG_VELNED:
      GPS_Info.GPS_speed         = _buffer.velned.speed_2d;  // cm/s
      GPS_Info.GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);  // Heading 2D deg * 100000 rescaled to deg * 10
      break;
    default:
      break;
    }
    return false;
  }
  bool GPS_UBLOX_newFrame(uint8_t data){
    bool parsed = false;
    printf("%c",data);
    switch(_step) {
      case 1:
        if (PREAMBLE2 == data) {
          _step++;
          break;
        }
        _step = 0;
      case 0:
        if(PREAMBLE1 == data) _step++;
        break;
      case 2:
        _step++;
        //_class = data;
        _ck_b = _ck_a = data;  // reset the checksum accumulators
        break;
      case 3:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _msg_id = data;
        break;
      case 4:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _payload_length = data;  // payload length low byte
        break;
      case 5:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _payload_length += (uint16_t)(data<<8);
        if (_payload_length > 512) {
          _payload_length = 0;
          _step = 0;
        }
        _payload_counter = 0;  // prepare to receive payload
      break;
      case 6:
        _ck_b += (_ck_a += data);  // checksum byte
        if (_payload_counter < sizeof(_buffer)) {
          _buffer.bytes[_payload_counter] = data;
        }
        if (++_payload_counter == _payload_length)
          _step++;
        break;
      case 7:
        _step++;
        if (_ck_a != data) _step = 0;  // bad checksum
      break;
      case 8:
        _step = 0;
        if (_ck_b != data)  break;  // bad checksum
        GPS_Info.GPS_Present = 1;
        if (UBLOX_parse_gps())  { parsed = true; }
    } //end switch
    return parsed;
}
bool isdigit(uint8_t p)
{
  if((p>=0x30)&&(p<0x3A))
    return true;
  else
    return false;
}
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 
bool GPS_NMEA_newFrame(char c) {
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;
  
    if (c == '$') {
      param = 0; offset = 0; parity = 0;
    } else if (c == ',' || c == '*') {
      string[offset] = 0;
      if (param == 0) { //frame identification
        frame = 0;
        if (string[0] == 'G' && string[1] == 'N' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
        if (string[0] == 'G' && string[1] == 'N' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
      } else if (frame == FRAME_GGA) {
        if (param == 2) {
          GPS_Info.GPS_coord[LAT] = GPS_coord_to_degrees(string);
        }
        else if (param == 3 && string[0] == 'S') 
          GPS_Info.GPS_coord[LAT] = -GPS_Info.GPS_coord[LAT];
        else if (param == 4)                     
          {GPS_Info.GPS_coord[LON] = GPS_coord_to_degrees(string);}
        else if (param == 5 && string[0] == 'W') 
          GPS_Info.GPS_coord[LON] = -GPS_Info.GPS_coord[LON];
        else if (param == 6) {
          fgps.GPS_FIX = (string[0]  > '0');
          GPS_Info.GPS_Fixed = fgps.GPS_FIX;
        }
        else if (param == 7) {
          GPS_Info.GPS_numSat = grab_fields(string,0);
        }
        else if (param == 9) {
          GPS_Info.GPS_altitude = grab_fields(string,0);
        }  // altitude in meters added by Mis
      } else if (frame == FRAME_RMC) {
          if (param == 7) {
            GPS_Info.GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
          else if (param == 8) {
            GPS_Info.GPS_ground_course = grab_fields(string,1);
          }                 //ground course deg*10 
      }
      param++; offset = 0;
      if (c == '*') checksum_param=1;
      else parity ^= c;
    } else if (c == '\r' || c == '\n') {
      if (checksum_param) { //parity checksum
        uint8_t checksum = hex_c(string[0]);
        checksum <<= 4;
        checksum += hex_c(string[1]);
        if (checksum == parity) frameOK = 1;
      }
      checksum_param=0;
    } else {
       if (offset < 15) string[offset++] = c;
       if (!checksum_param) parity ^= c;
    }
    if (frame) GPS_Info.GPS_Present = 1;
    return frameOK && (frame==FRAME_GGA);
}

bool GPS_newFrame(char c) {
	return GPS_NMEA_newFrame(c);
}
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (fabs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}
void GPS_reset_home_position(void) {
	float Euler[3];
	nvtGetEulerRPY(Euler);
  if (fgps.GPS_FIX && GPS_Info.GPS_numSat >= 5) {
      GPS_Info.GPS_home[LAT] = GPS_Info.GPS_coord[LAT];
      GPS_Info.GPS_home[LON] = GPS_Info.GPS_coord[LON];
      GPS_calc_longitude_scaling(GPS_Info.GPS_coord[LAT]);  //need an initial value for distance and bearing calc

    nav_takeoff_bearing = Euler[2];//save takeoff heading
    //Set ground altitude
    fgps.GPS_FIX_HOME = 1;
  }
}
//It was mobed here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}
////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void) {
  int32_t temp;
  temp = target_bearing - original_target_bearing;
  temp = wrap_18000(temp);
  return (fabs(temp) > 10000);   // we passed the waypoint by 100 degrees
}
////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(dLat*dLat + dLon*dLon) * 1.113195;
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}
//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(void){
  //static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0f/dTnav;
    actual_speed[_X] = (float)(GPS_Info.GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_Info.GPS_coord[LAT]  - last[LAT])  * tmp;

#if !defined(GPS_LEAD_FILTER) 
    actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
    actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;
  
    speed_old[_X] = actual_speed[_X];
    speed_old[_Y] = actual_speed[_Y];

#endif
  }
  init=1;

  last[LON] = GPS_Info.GPS_coord[LON];
  last[LAT] = GPS_Info.GPS_coord[LAT];

#if defined(GPS_LEAD_FILTER)
  GPS_coord_lead[LON] = LeadFilterX_get_position(GPS_Info.GPS_coord[LON], actual_speed[_X], GPS_LAG);
  GPS_coord_lead[LAT] = LeadFilterY_get_position(GPS_Info.GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
#endif

}
////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}
////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(void) {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;
  
  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
    rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error

    nav[axis]      =
        get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
       +get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = constrain(d, -2000, 2000);
    // get rid of noise
    if(fabs(actual_speed[axis]) < 50) d = 0;

    nav[axis] +=d;
    nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
  }
}
int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
  return ang;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line 
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void) {
  if (fabs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {  // If we are too far off or too close we don't do track following
    float temp = (target_bearing - original_target_bearing) * RADX100;
    crosstrack_error = sin(temp) * (wp_distance * CROSSTRACK_GAIN);  // Meters we are off track line
    nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
    nav_bearing = wrap_36000(nav_bearing);
  }else{
    nav_bearing = target_bearing;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(uint16_t max_speed) {
  float trig[2];
  uint8_t axis;
  float temp;
  // push us towards the original track
  GPS_update_crosstrack();

  // nav_bearing includes crosstrack
  temp = (9000l - nav_bearing) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  for (axis=0;axis<2;axis++) {
    rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis]; 
    rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
    // P + I + D
    nav[axis]      =
        get_P(rate_error[axis],                        &navPID_PARAM)
       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
       +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

    nav[axis]      = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    poshold_ratePID[axis].integrator = navPID[axis].integrator;
  }
}


////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow 
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  // max_speed is default 400 or 4m/s
  if(_slow){
    max_speed = min(max_speed, wp_distance / 2);
    //max_speed = max(max_speed, 0);
  }else{
    max_speed = min(max_speed, wp_distance);
    max_speed = max(max_speed, NAV_SPEED_MIN);  // go at least 100cm/s
  }

  // limit the ramp up of the speed
  // waypoint_speed_gov is reset to 0 at each new WP command
  if(max_speed > waypoint_speed_gov){
    waypoint_speed_gov += (int)(100.0f * dTnav); // increase at .5/ms
    max_speed = waypoint_speed_gov;
  }
  return max_speed;
}
uint16_t GPSBufferAvailable()
{
  /*uint16_t tmp;
  tmp = g_u32comRtail;
  if(g_u32comRhead != tmp)*/
    return g_u32comRbytes;
 /* else
    return 0;*/
    
}
char GPSBufferRead()
{
  char u8InChar;
  //int test;
  u8InChar = g_u8RecData[g_u32comRhead];
  g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
  //test = g_u32comRbytes;
  UART_DisableInt(UART1, UART_INTEN_RDAIEN_Msk);
  g_u32comRbytes--;
  UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk);
  //printf("<%d\n",g_u32comRbytes);
 // if(g_u32comRbytes==0xFFFFFFFF)
  //  printf("Error:0x%x\n",test);
  return u8InChar;
}
void GPS_NewData(void) {
	static uint32_t nav_loopTimer;
	uint32_t dist;
	int32_t  dir;
	uint8_t axis;
  int16_t speed;
	uint16_t c = GPSBufferAvailable();
	while (c--) {
		if (GPS_newFrame(GPSBufferRead())) {
			if (GPS_Info.GPS_update == 1) 
				GPS_Info.GPS_update = 0;
			else 
				GPS_Info.GPS_update = 1;
			
			if (fgps.GPS_FIX && GPS_Info.GPS_numSat >= 5) {
				if (!checkArm()) {
					fgps.GPS_FIX_HOME = 0;
				}
				if (!fgps.GPS_FIX_HOME && checkArm()) {
					GPS_reset_home_position();
				}
				//Apply moving average filter to GPS data
#if defined(GPS_FILTERING)
				GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
				for (axis = 0; axis< 2; axis++) {
					GPS_data[axis] = GPS_Info.GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
					GPS_degree[axis] = GPS_data[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
					// How close we are to a degree line ? its the first three digits from the fractions of degree
					// later we use it to Check if we are close to a degree line, if yes, disable averaging,
					fraction3[axis] = (GPS_data[axis]- GPS_degree[axis]*10000000) / 10000;
					GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
					GPS_filter[axis][GPS_filter_index] = GPS_data[axis] - (GPS_degree[axis]*10000000); 
					GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
					GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
					if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
						if ( fraction3[axis]>1 && fraction3[axis]<999 ) 
							GPS_Info.GPS_coord[axis] = GPS_filtered[axis];
					}
				}
#endif
				//Time for calculating x,y speed and navigation pids
				dTnav = (float)(getTickCount() - nav_loopTimer)/ 10000.0f;
				nav_loopTimer = getTickCount();
				// prevent runup from bad GPS
				dTnav = min(dTnav, 1.0f);  
				//calculate distance and bearings for gui and other stuff continously - From home to copter
				GPS_distance_cm_bearing(&GPS_Info.GPS_coord[LAT],&GPS_Info.GPS_coord[LON],&GPS_Info.GPS_home[LAT],&GPS_Info.GPS_home[LON],&dist,&dir);
				GPS_Info.GPS_distanceToHome = dist/100;
				GPS_Info.GPS_directionToHome = dir/100;
				
				if (!fgps.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
					GPS_Info.GPS_distanceToHome = 0;
					GPS_Info.GPS_directionToHome = 0;
				}
        //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
        GPS_calc_velocity();        
        if (fgps.GPS_HOLD_MODE || fgps.GPS_HOME_MODE){    //ok we are navigating 
          //do gps nav calculations here, these are common for nav and poshold  
          #if defined(GPS_LEAD_FILTER)
          GPS_distance_cm_bearing(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
          GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
          #else
          GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
          GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
          #endif
          switch (nav_mode) {
            case NAV_MODE_POSHOLD: 
              //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
              GPS_calc_poshold();
              break;
            case NAV_MODE_WP:
              speed = GPS_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation 
              // use error as the desired rate towards the target
              //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
              GPS_calc_nav_rate(speed);
              //Tail control
              if (NAV_CONTROLS_HEADING) {
                if (NAV_TAIL_FIRST) {
                  magHold = wrap_18000(nav_bearing-18000)/100;
                } else {
                  magHold = nav_bearing/100;
                }
              }
              // Are we there yet ?(within 2 meters of the destination)
              if ((wp_distance <= GPS_wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
                nav_mode = NAV_MODE_POSHOLD;
                if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
              } 
              break;               
          }
        } //end of gps calcs  
      }
    }
  }
}
void GPS_UART_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART1);

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
                //printf(">%d",g_u32comRbytes);
            }
        }
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    GPS_UART_HANDLE();
}
void setupGpsUART()
{
#ifdef GPS_UART
	/* Enable peripheral clock */
	CLK_EnableModuleClock(UART1_MODULE);
	/* Peripheral clock source */
	CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
#if (BOARD_CODE == 200)
  /* Set PE multi-function pins for UART0 RXD, TXD */
	SYS->GPA_MFPL = SYS_GPA_MFPL_PA1MFP_UART1_RXD | SYS_GPA_MFPL_PA0_UART1_TXD;
#else
	/* Set PE multi-function pins for UART0 RXD, TXD */
	SYS->GPE_MFPH = SYS_GPE_MFPH_PE13MFP_UART1_RXD | SYS_GPE_MFPH_PE12MFP_UART1_TXD;
#endif
	/* Reset UART module */
	SYS_ResetModule(UART1_RST);
	/* Configure UART0 and set UART0 Baudrate */
	UART_Open(UART1, GPS_BAUD);
  UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk);
#endif
}
void setupGPS() 
{
  uint8_t i;
	//GPS_write(UBLOX_57600,sizeof(UBLOX_57600),5);
  //UART_Open(UART1, 57600);
	//DelayMsec(200);
	//GPS_write(UBLOX_INIT, sizeof(UBLOX_INIT),5);
  setupGpsUART();
  for(i=0; i<5; i++) {
    GPS_NewData();
    DelayMsec(100);
  }
  if(GPS_Info.GPS_Present)
    printf("GPS OK\n");
  else
    printf("GPS ERROR\n");
}
void GPSCommandProcess()
{
	GPS_NewData();
  //printf("GPS num:%d, LAT:%d, LON:%d\n",GPS_Info.GPS_numSat, GPS_Info.GPS_coord[LAT], GPS_Info.GPS_coord[LON]);
	/*while (GPSBufferAvailable())
	{
		char start=GPSBufferRead();
		printf("%c",start);
	}*/
}
GPS_Info_T* GetGPSInfo()
{
  return &GPS_Info;
}
