/*
project_QuadX_2560 GY86_PIDAuto_V1  
1. Automatic  Takeoff 
2. Landing
3. GPS Position Hold
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

date: 22-12-2557(2014)  V.1 QuadX_2560 GY86_PIDAuto_V1
date: 28-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V2   ,Write P-PID Controller
date: 28-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V3   ,Ultrasonic and baro ,,Ultrasonic max 0.8 m change to Baro
date: 30-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V4   ,tuning P-PID Controller
date: 31-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V5   ,tuning Altitude Hold , Position Hold

support:  Board 2560  GY86
• Atmega2560
• MPU6050 Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• MS561101BA Barometer
• HMC5883L Magnetometer //400kHz
• GPS BS-300
• Ultrasonic Sensor Module
• ESC Dargon 30A
• motor sunny 2212 1400kv 

Quad4-X
---------motor---------
int MOTOR_FrontL_PIN = 2;
int MOTOR_FrontR_PIN = 5;
int MOTOR_BackL_PIN = 6;
int MOTOR_BackR_PIN = 3;

----------rx-----------           
CPPM pin A8
*/
#include <Arduino.h>
#include <Wire.h>
#include "MS561101BA.h"
#include "config.h"
#include "multi_rxPPM2560.h"
#include "mpu6050.h"
#include "ahrs_tin.h"
#include "Control_PID.h"
#include "motorX4.h"
#include "GPS_multi.h"
#include "Ultrasonic.h"

float getAltitude(float pressure2, float temperature2)
{
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return log(sea_press/pressure2) * (temperature2+273.15) * 29.271267f; // in meter 
  //return ((pow((sea_press/pressure),1/5.257)-1.0)*(temperature+273.15))/0.0065;
}
void pushAvg(float val)
{
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}
float getAvg(float * buff, int size)
{
  float sum=0.0;
  for(int i=0;i<size;i++)
  {
    sum += buff[i];
  }
  return sum/size;
}
void setup()
{
  Serial.begin(57600);//38400
  pinMode(13, OUTPUT);pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  digitalWrite(13, HIGH);
  //Serial1.begin(115200);//CRIUS Bluetooth Module pin code 0000
  //Serial3.begin(38400);//3DR Radio Telemetry Kit 433Mhz
  configureReceiver();//find multi_rx.h
  motor_initialize();//find motor.h
  ESC_calibration();//find motor.h
  GPS_multiInt();
  Wire.begin();
  delay(1);
  mpu6050_initialize();
  delay(1); 
  MagHMC5883Int();
  delay(1); 
  digitalWrite(13, HIGH);
  baro.init(MS561101BA_ADDR_CSB_LOW);
  UltrasonicInt();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 
  delay(1);
      for(uint8_t i=0; i<50; i++) 
    {
     mpu6050_Gyro_Values();
     mpu6050_Accel_Values();
     Mag5883Read();
     UltrasonicRead();
     temperature = baro.getTemperature(MS561101BA_OSR_4096);
     presser = baro.getPressure(MS561101BA_OSR_4096);
     pushAvg(presser);
     delay(20);
    }
    //Altitude_Ground = Altitude_baro/10.0;
    sea_press = presser + 0.11;//presser 1003.52
    Serial.print("presser ");Serial.println(sea_press);
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  ahrs_initialize();//ahrs.h
  setupFourthOrder();
  RC_Calibrate();//"multi_rxPPM2560.h"
  Serial.print("TK_Quadrotor_Run_Roop_100Hz");Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop()
{
    while(Serial2.available())
   {
     //digitalWrite(30, LOW);//C
     char byteGPS1=Serial2.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
     GPS_LON1 = GPS_coord[LON]/10000000.0;
     //if(GPS_FIX  == 1 && GPS_Present == 1){
        //digitalWrite(30, HIGH);
       //}
     }//end gps
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 3 samples = 2760 us  && gyroSamples < 5  && gyroSamples < 5
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 10001; 
   }   
    if (Dt_roop >= 10000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Get_accel();
      mpu6050_Get_gyro();
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
////////////////Low pass filter/////////////////////////////////
      AccXf = AccX;
      AccYf = AccY;
      AccZf = AccZ;
      //AccXf = AccXf + (AccX - AccXf)*15.6*G_Dt;//29.6 15.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      //AccYf = AccYf + (AccY - AccYf)*15.6*G_Dt;//15.4
      //AccZf = AccZf + (AccZ - AccZf)*15.6*G_Dt;//15.4
      ///////////////////Filter FourthOrder ///////////////////////////////////////
    Accel[XAXIS] = AccX;
    Accel[YAXIS] = AccY;
    Accel[ZAXIS] = AccZ;
    for (int axis = XAXIS; axis <= ZAXIS; axis++) {
      filteredAccel[axis] = computeFourthOrder(Accel[axis], &fourthOrder[axis]);//"ahrs_tin.h"
    }
    
    //AccXf = filteredAccel[XAXIS];
    //AccYf = filteredAccel[YAXIS];
    //AccZf = filteredAccel[ZAXIS];
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
      //ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);
      //x_angle = x_angle + (GyroXf*RAD_TO_DEG*G_Dt);
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);
      
///Observer velocity vx vy vz kalman//accelerometer and GPS/////////////////////////////////////////////////////
     //GPS Speed Low Pass Filter
     //actual_speedXf = actual_speedXf + (actual_speedX - actual_speedXf)*10.2*G_Dt;//5.2 10.4  //cm/s  +-400 cm/s
     //actual_speedYf = actual_speedYf + (actual_speedY - actual_speedYf)*10.2*G_Dt;//5.2 10.17
      //GPS_LAT1Lf = GPS_LAT1Lf + (GPS_LAT1lead - GPS_LAT1Lf)*8.2521*G_Dt;//12.5412
      //GPS_LON1Lf = GPS_LON1Lf + (GPS_LON1lead - GPS_LON1Lf)*8.2521*G_Dt;
      float temp_vx = accrX_Earth*100.0 + (_velocity_north - vx_hat2)*13.2;//2.15  1.5 6.5      vx_hat = vx_hat + temp_vx*G_Dt;
      vx_hat2 = vx_hat2 + temp_vx*G_Dt;
      vx_hat = constrain(vx_hat2, -400, 400);//+-400 cm/s
      applyDeadband(vx_hat, 4.5);//10.5 cm/s
      float temp_vy = accrY_Earth*100.0 + (_velocity_east - vy_hat2)*13.2;//2.15  1.5 6.5
      vy_hat2 = vy_hat2 + temp_vy*G_Dt;
      vy_hat = constrain(vy_hat2, -400, 400);
      applyDeadband(vy_hat, 4.5);//10.5 cm/s
//////////////////////////////////////////////////     
      //Observer hz kalman , GPS_hz , GPS_vz
      float temp_vz = accrZ_Earth + (baro_vz - vz_hat2)*0.45 + (Altitude_Baro_ult - Altitude_hat)*3.72;//4.72 ,,0.95
      vz_hat2 = vz_hat2 + temp_vz*G_Dt;
      vz_hat = constrain(vz_hat2, -4, 4);//+-4 m/s
      applyDeadband(vz_hat, 0.04);//+-0.01 m/s
      float temp_hz = vz_hat + (Altitude_Baro_ult - Altitude_hat)*6.72;//4.72 12.54 4.5
      Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      //Altitude_hat = constrain(Altitude_hat, 0.0, 1.2);//1.5 m By Ultrasonic
      //vz = vz + accrZ_Earth*G_Dt;
          baro_vz = (Altitude_hat - baro_vz_old2)/0.02;//Diff Altitude
          baro_vz_old2 = baro_vz_old;
          baro_vz_old = Altitude_hat;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PID Control///////////
     Control_PIDRate();//Control_PID.h
//////Out motor///////////
//armed = 1;
     motor_Mix();//"motor.h"
/////////////////////////
     motor_command(); 
////////end Out motor//////
 if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  computeRC();//multi_rx.h
  //failsafeCnt++;
  //Fail_Safe();
       if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0 && abs(ahrs_p) < 10 && abs(ahrs_r) < 10)//+- 10 deg, ARM 
            {
                armed = 1;
                digitalWrite(31, HIGH);//B
                setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
            }
            if (CH_RUD < MINCHECK && armed == 1) //DISARM
            {
                armed = 0;
                digitalWrite(31, LOW);
                Altitude_Ground = Altitude_baro;
                Altitude_II = 0.17;
            }
            if (CH_RUD < MINCHECK && armed == 0 && CH_ELE > MAXCHECK) //Mag_Calibrate
            {
              Mag_Calibrate();//#include "mpu6050.h"
            }
        }//end  ARM and DISARM your helicopter///////////////      
}//end roop 50 Hz 
presser = baro.getPressure(MS561101BA_OSR_4096);//read 100 Hz
pushAvg(presser);//
         if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
          UltrasonicRead();//"Ultrasonic.h"
          Mag5883Read();//"mpu6050.h"  
          presserf = getAvg(movavg_buff, MOVAVG_SIZE);
          Altitude_baro = getAltitude(presserf,temperature);//Altitude_Ground
          Altitude_barof = Altitude_baro - Altitude_Ground + Altitude_II;
        }//end roop 20 Hz
         if (frameCounter % TASK_10HZ == 0)// 10 Hz task (100 ms)
        {
          //presserf = getAvg(movavg_buff, MOVAVG_SIZE);
          //Altitude_baro = getAltitude(presserf,temperature);//Altitude_Ground
          //Altitude_barof = Altitude_baro - Altitude_Ground + Altitude_II;
          Chack_Command();//Control pid
          Altitude_sonaold = Altitude_sonaf;
          Automatictakeland();
        }//end roop 10 Hz
        if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
           Cal_GPS();
           //GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
           if(Mode == 2 && GPS_FIX == 1)//Position_Hold  if(Mode == 2 || Mode == 3 && GPS_FIX == 1)
             {
               GPS_calc_positionhold();//Control_PID.h
             }
             else
             {
               GPS_I_LAT = 0.0;
               GPS_I_LON = 0.0;
               Control_XBf = 0.0;
               Control_YBf = 0.0;
               error_LAT = 0.0;
               error_LON = 0.0;
               target_LAT = GPS_LAT1f;//GPS_LAT_Hold
               target_LON = GPS_LON1f;//GPS_LON_Hold
             }
        }//end gps
         if (frameCounter % TASK_20HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            //Serial.print(AUX_3);Serial.print("\t"); 
            //Serial.print(AUX_4);Serial.print("\t"); 
            //Serial.print(failsafeCnt);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZf);Serial.print("\t");  
            //Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            
            //Serial.print(c_acc_x);Serial.print("\t");
            //Serial.print(c_acc_y);Serial.print("\t");
            //Serial.print(c_acc_z);Serial.print("\t");
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX);Serial.print("\t");
            //Serial.print(GPS_LAT1,9);Serial.print("\t");
            //Serial.print(GPS_LAT1lead,9);Serial.print("\t"); 
            Serial.print(GPS_LAT1f,9);Serial.print("\t");
                        
            //Serial.print(GPS_LON1,9);Serial.print("\t");
            Serial.print(GPS_LON1f,9);Serial.print("\t");
            //Serial.print(GPS_LON1f2,9);Serial.print("\t");
            //Serial.print(GPS_LON1lead,9);Serial.print("\t");
            //Serial.print(error_LAT);Serial.print("\t");
            //Serial.print(error_LON);Serial.print("\t");
            //Serial.print(GPS_speed);Serial.print("\t");//cm/s
            //Serial.print(GPS_ground_course);Serial.print("\t");//deg
            
            Serial.print(_velocity_north);Serial.print("\t");
            //Serial.print(actual_speedX);Serial.print("\t");
            //Serial.print(actual_speedXf);Serial.print("\t");
            //Serial.print(vx_hat);Serial.print("\t");
            Serial.print(_velocity_east);Serial.print("\t");
            //Serial.print(actual_speedY);Serial.print("\t");
            //Serial.print(actual_speedYf);Serial.print("\t");
            //Serial.print(vy_hat);Serial.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            
            //Serial.print(temperature);Serial.print("\t");
            //Serial.print(presser*100);Serial.print("\t");
            //Serial.print(presserf*100);Serial.print("\t");
            //Serial.print(Altitude_baro);Serial.print("\t");
            //Serial.print(Altitude_barof);Serial.print("\t");
            //Serial.print(Altitude_sona);Serial.print("\t");
            //Serial.print(Altitude_sonaf);Serial.print("\t");
            //Serial.print(Altitude_Baro_ult);Serial.print("\t");
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial.print(Altitude_II);Serial.print("\t");
            
            //Serial.print(baro_vz*10);Serial.print("\t");
            //Serial.print(vz_sonaf*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 

            //Serial.print(vz_hat);Serial.print("\t");
            //Serial.print(DCM10);Serial.print("\t");
            //Serial.print(DCM11);Serial.print("\t");
            //Serial.print(DCM12);Serial.print("\t");
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ);Serial.print("\t");
            //Serial.print(AccZf2,3);Serial.print("\t");
            //Serial.print(AccZf3,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");

            
            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(courseRads);Serial.print("\t"); 
            //Serial.print(ahrs_r);Serial.print("\t");
            //Serial.print(ahrs_p);Serial.print("\t");  
            //Serial.print(ahrs_y);Serial.print("\t");  
            //Serial.print(Heading);Serial.print("\t");
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(u_roll);Serial.print("\t");
            //Serial.print(u_pitch);Serial.print("\t");
            //Serial.print(u_yaw);Serial.print("\t");
            
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(GPSI_LAT);Serial.print("\t");
            //Serial.print(motor_FrontL);Serial.print("\t");
            //Serial.print(motor_FrontR);Serial.print("\t");
            //Serial.print(motor_BackL);Serial.print("\t");
            //Serial.print(motor_BackR);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            //Serial.print(gyroSamples2);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            time_auto++;
            temperature = baro.getTemperature(MS561101BA_OSR_4096);
            Remote_TrimACC();//motor.h
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
