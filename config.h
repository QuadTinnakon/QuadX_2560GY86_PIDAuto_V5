/*
project_QuadX_2560 GY86_PIDAuto_V1  
1. Automatic  Takeoff 
2. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
//The type of multicopter   
//#define Quad_X
#define PPM
///////////////Mode///////////////////////////
#define AltHold 1500
#define PositionHold 1700
#define Auto 1960
int Mode = 0;
//Mode 0 = Stabilize
//Mode 1 = Altitude Hold
//Mode 2 = Position Hold ,Loiter
//Mode 3 = Automatic  Takeoff ,Landing
#define MINTHROTTLE 1064 //1090
#define MAXTHROTTLE 1850
#define MINCOMMAND 1000
#define MAXCOMMAND 1850
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900
/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// Automatic take-off and landing 
#define h_control 0.9  //0.6 0.9 meter
//////////////////////////////////////////////////////////////////////
//P-PID-------------Rate
float Kp_rateRoll = 1.02;//2.78 1.18 5.28
float Ki_rateRoll = 0.82;//2.75
float Kd_rateRoll = 0.065;//0.085 0.025 - 0.045

float Kp_ratePitch = 1.02;//1.18 5.28
float Ki_ratePitch = 0.82;//2.75 0.5 - 2.8
float Kd_ratePitch = 0.065;//0.078 0.025 - 0.045

float Kp_rateYaw = 2.75;//3.75 5.75 1.75 - 3.450  350.0
float Ki_rateYaw = 0.85;//3.65  2.95
float Kd_rateYaw = 0.065;//0.035 0.065

//PID--------------Stable
float Kp_levelRoll= 5.2;//4.2 6.2 7.8 9.2 
//float Ki_levelRoll= 0.00;//0.0
//float Kd_levelRoll= 0.00;//0.0

float Kp_levelPitch= 5.2;//4.2 6.2 9.2 
//float Ki_levelPitch= 0.00;
//float Kd_levelPitch= 0.00;

float Kp_levelyaw= 4.5;//4.2

//state feedback--------------Altitude
float Kp_altitude = 165.2;//145.2  165.0 425.2 365 165.0
float Ki_altitude = 56.13;//0.018 2.5,0.0
float Kd_altitude = 280.5;//250 280.5 315.5 120
float Ka_altitude = 0.5;//32.5 38.5 41.5 35 25 - 45

//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
#define tarremote 0.025  //0.092 slow 0.12 0.02 0.08 remote 
#define tar 0.015 //0.011 0.012 0.015
//////////////////////////////////////////////////

//PID GPS////////////////////////////////////////////
float Kp_gps = 0.45;//0.085 0.15 0.101 2.101 5.101
float Ki_gps = 0.085;//0.15
float Kd_gps = 1.05;//1.9 4.3 0.35 1.35 3.35
float Kp_speed = 0.15;//0.25 0.35 0.095 min 0.15

//GPS //สตาร์ท//////////////////////////////////////
float GPS_LAT_HOME = 13.867000579 ;//13.867000579  13.867021560  13.867017745      14.907173156
float GPS_LON_HOME = 100.483291625; //100.483291625 100.483261108  100.483276367  100.206214904
//ลงจอด
float waypoint1_LAT = 13.875096;//F, 13.875096, 100.484546     ,R     13.867492, 100.501004
float waypoint1_LON = 100.484546;//B, 13.857347, 100.483344   ,L     13.866868, 100.473152
//
float waypoint2_LAT = 13.867051124;
float waypoint2_LON = 100.483238220;//13.866970, 100.483240
//
float waypoint3_LAT = 13.867051124;
float waypoint3_LON = 100.483238220;
//
float waypoint4_LAT = 13.867051124;
float waypoint4_LON = 100.483238220;

//Parameter system Quadrotor
#define m_quad 1.34 //kg 1.56
#define L_quad 0.25 //m
float I_x = 0.002515;
float I_y = 0.002521;
float I_z = 0.002552;
float I1 = 105.0;//0.55
float I2 = -105.0;
float I3 = 105.0;
float Nm_to_PWM = 457.17;//509.68 , 407.74
////////////////////////////////////////////////////////////////////
//Accelerometer calibration constants; use the Calibrate example from
int A_X_MIN = -4265;    //
int A_X_MAX = 3978;     //
int A_Y_MIN = -4048;    //
int A_Y_MAX = 4113;     //
int A_Z_MIN = -4105;    //
int A_Z_MAX = 4211;     //
////////////////////////////////////////////////////////////////////
//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board
int M_X_MIN = -270;    //-654  -693   -688
int M_X_MAX = 585;     //185   209    170
int M_Y_MIN = -600;    //-319  -311   -310
int M_Y_MAX = 260;     //513   563    546
int M_Z_MIN = -425;    //-363  -374   -377
int M_Z_MAX = 285;     //386   429    502
////////////////////////////////////////////////////////////////////
//Observer hz
float Altitude_Hold = 0.0;
float Altitude_hat=0.0;//Observer hx
float vx_hat=0.0;
float vx_hat2=0.0;
float vy_hat=0.0;
float vy_hat2=0.0;
float vz_hat=0.0;
float vz_hat2=0.0;
float h=0.0;
float seth=0.0;//set control
float uthrottle=0.0;
float uAltitude = 1000.0;
float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrZ_Earthf = 0.0;
//float vz = 0.0;

//GPS
float GPS_LAT1 = 0.0;
float GPS_LON1 = 0.0;
float GPS_LAT1f = 0.0;
float GPS_LON1f = 0.0;
float GPS_LAT1Lf = 0.0;
float GPS_LON1Lf = 0.0;
float GPS_LAT1lead = 0.0;
float GPS_LON1lead = 0.0;
float GPS_speed = 0.0;
float actual_speedX = 0.0;
float actual_speedY = 0.0;
float actual_speedXf = 0.0;
float actual_speedYf = 0.0;
float actual_speedXold = 0.0;
float actual_speedYold = 0.0;
float _last_velocityX = 0.0;
float _last_velocityY = 0.0;
float GPS_LAT1_old = GPS_LAT_HOME;
float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0;
float Control_YEf = 0.0;
float Control_XBf = 0.0;
float Control_YBf = 0.0;
float target_LAT = 0.0;
float target_LON = 0.0;
byte currentCommand[23];
byte Read_command = 0;
float GPS_hz = 0.0;
float GPS_vz = 0.0;
float GPS_ground_course2 = 0.0;
float GPS_Distance = 0.0;
float error_LAT = 0.0;
float error_LON = 0.0;  
float GPS_I_LAT = 0.0;
float GPS_I_LON = 0.0;  
uint8_t GPS_filter_index = 0;
float GPS_SUM_LAT[5];
float GPS_SUM_LON[5];
//float courseRads = 0.0;

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_2HZ 50
#define TASK_1HZ 100
#define RAD_TO_DEG 57.295779513082320876798154814105

//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0;

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;

uint8_t frameCounter = 0;
uint8_t timeLanding = 0;
uint8_t timeOff = 0;
byte armed = 0;
float G_Dt = 0.01; 

long Dt_sensor = 1000;
long Dt_roop = 10000;

int Status_LED = LOW;
int ESC_calibra = 0;

//Baro
MS561101BA baro = MS561101BA();
#define  MOVAVG_SIZE 20  //30
float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;
float sea_press=1013.25;
float temperature;
float presser;
float presserf;
float Altitude_baro=0.0;
float Altitude_barof=0.0;
float Altitude_Ground=0.0;
float baro_vz = 0.0;
float baro_vz_old = 0.0;
float baro_vz_old2 = 0.0;
