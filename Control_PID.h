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
int u_roll = 0;
int u_pitch = 0;
int u_yaw = 0;
float roll_I_rate = 0.0;
float roll_D_rate = 0.0;
float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float error_rollold = 0.0;
float error_rate_rollold = 0.0;
float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;
float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

//Automatic take-off and landing
int time_auto = 0;
float  h_counter = 0.03;//0.08
float h_counter_old = 0.0;
float Vz_Hold = 0.0;
float hz_I = 0.0;
float err_hz = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;

void Control_PIDRate(){
//P-PID By tinnakon
//if(AUX_1 >= 1300)//Flight Modes 2.Stabilize
// ROLL CONTROL P-PID///////////
  float setpoint_roll = ((CH_AILf-CH_AIL_Cal)*0.085) + Control_YBf;//0.12 max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 1.5);//1.2
  setpoint_rate_roll = (0.065*setpoint_rate_roll/(0.065+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.065+G_Dt));//Diff remote
  setpoint_rollold = setpoint_roll;
  setpoint_rate_roll = constrain(setpoint_rate_roll, -80, 80);//+-80 deg/s
  float error_roll = setpoint_roll - ahrs_r;//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll + error_roll*Kp_levelRoll  - GyroXf*RAD_TO_DEG;
  roll_I_rate += error_rate_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -50, 50);//+-150
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate;
  u_roll = constrain(u_roll, -200, 200);//+-300 120
  
////// PITCH CONTROL  P-PID///////////
  float setpoint_pitch = ((CH_ELEf-CH_ELE_Cal)*-0.085) + Control_XBf;//max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 1.5);//1.2
  setpoint_rate_pitch = (0.065*setpoint_rate_pitch/(0.065+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.065+G_Dt));//Diff remote
  setpoint_pitchold = setpoint_pitch;
  setpoint_rate_pitch = constrain(setpoint_rate_pitch, -80, 80);//+-80
  float error_pitch = setpoint_pitch - ahrs_p;//ahrs_p*RAD_TO_DEG
  float error_rate_pitch = setpoint_rate_pitch + error_pitch*Kp_levelPitch - GyroYf*RAD_TO_DEG;
  pitch_I_rate += error_rate_pitch*Ki_ratePitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -50, 50);//+-150
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  error_rate_pitchold = error_rate_pitch;
  u_pitch = Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate;
  u_pitch = constrain(u_pitch, -200, 200);//+-300 120
////// YAW CONTROL PID///////////
  float setpoint_rate_yaw = (CH_RUDf-CH_RUD_Cal)*0.55;//0.4 0.35
  applyDeadband(setpoint_rate_yaw, 7.1);//6.5
  if(abs(setpoint_rate_yaw) > 0.1){
   setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
  }
  float error_yaw = 0.0 - Heading;
  float error_rate_yaw = setpoint_rate_yaw + error_yaw*Kp_levelyaw - GyroZf*RAD_TO_DEG;
  yaw_I_rate += error_rate_yaw*Ki_rateYaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -50, 50);//+-100
  yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(tar+G_Dt));
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate;
  u_yaw = constrain(u_yaw, -120, 120);//+-150
 ////Altitude State feedback control//////////////////////////////////////////////////////////////////////////////////////////////////////  
  if(Mode == 1 || Mode == 2)//Altitude Hold,
  {
    err_hz = Altitude_Hold - Altitude_hat;
    err_hz = constrain(err_hz, -2, 2);//+-2 m
    //applyDeadband(err_hz, 0.09);//nois 0.2 m
    float error_Vz = 0.0 - vz_hat;
    float error_hII = err_hz*0.55 + error_Vz;
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -120, 120);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) - (accrZ_Earth*Ka_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad/cos_rollcos_pitch;//  /cos_rollcos_pitch
    uthrottle = constrain(uthrottle, -200, 200);
  }
  else if(Mode == 3)//Automatic  Takeoff, Landing
  {
    err_hz = h_counter - Altitude_hat;
    err_hz = constrain(err_hz, -2, 2);//+-2 m
    //applyDeadband(err_hz, 0.09);//nois 0.2 m
    float error_Vz = Vz_Hold - vz_hat;
    float error_hII = err_hz*0.55 + error_Vz;
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -120, 120);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) - (accrZ_Earth*Ka_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad/cos_rollcos_pitch;//  /cos_rollcos_pitch
    uthrottle = constrain(uthrottle, -200, 200);
  }
  else
  {
    hz_I = 0.0;
    uthrottle = 0.0;
    Altitude_Hold = Altitude_hat;
  }//end Altitude Hold
uAltitude = CH_THR + uthrottle;//m*g = 10.8 N = 
}

void Automatictakeland(){
       if(h_counter == 0.0 && AUX_1 < (Auto-10))
      {
        time_auto = 0;
        //target_LAT = GPS_LAT1;//GPS_LAT_HOME
        //target_LON = GPS_LON1;//GPS_LON_HOME
      }
//////////////////////////////////////////////////////////
 //Altitude control and 1 waypoint navigation
  if(Mode == 3 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 1;
     if(Altitude_hat >= h_control && endAuto == 1)//waypoint1
    {
      //target_LAT = waypoint1_LAT;
      //target_LON = waypoint1_LON;
    }
     if(time_auto > 6 && endAuto == 1)//1m Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 20)//20 relay 2 s Landing
      {
        takeoff = 0;
      }
    }
  }
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
  } 
////////////////////////////////////////////////////////////////// 
         if (time_auto > 6 && endAuto == 0) //End waypoint quadrotor
        {
          timeOff++;
          if(timeOff > 10)//relay 1 s timeOff
          {
           armed = 0;
          }
        }  
////////////////////////////////////////////////////////////
       if(h_counter < h_control && takeoff == 1)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.023;//0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
      }
       if(takeoff == 0)//landing
      {
        h_counter = h_counter - 0.023; //ramp input hz  landing
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
        if(h_counter <= 0.03)
        {
         h_counter = 0.0;
         endAuto = 0;
        }
      }
}

 void GPS_distance_m_bearing(float lat1, float lon1, float lat2, float lon2, float alt){
  float a, R, c, d, dLat, dLon;
  lon1=lon1/RAD_TO_DEG;
  lat1=lat1/RAD_TO_DEG;
  lon2=lon2/RAD_TO_DEG;
  lat2=lat2/RAD_TO_DEG;
  R=6371000.0;    //m raio da terra 6371km
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  GPS_ground_course2 = a*RAD_TO_DEG;
  //if (yaw<0) yaw=360+yaw;
//calculo da distancia entre modelo e home
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  c = 2* asin(sqrt(a));  
  d = R * c;
  //alt=alt-Alt_Home;
  //pitch=atan(alt/d);
  //pitch=pitch*360/(2*PI);
  GPS_Distance = sqrt(alt*alt+d*d);
}
  void GPS_calc_positionhold(){
          error_LAT = (target_LAT - GPS_LAT1f)*6371000.0; // X Error cm
          error_LON = (target_LON - GPS_LON1f)*6371000.0;// Y Error   cm
          error_LAT = constrain(error_LAT,-500,500);//200 = +-2 m
          error_LON = constrain(error_LON,-500,500);
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          target_speedLAT = constrain(target_speedLAT,-100,100);//+-100 cm/s = 1m/s
          target_speedLON = constrain(target_speedLON,-100,100);

          float error_rate_LAT = target_speedLAT - vx_hat;
          float error_rate_LON = target_speedLON - vy_hat;
          error_rate_LAT = constrain(error_rate_LAT,-300,300);//+-200 cm/s
          error_rate_LON = constrain(error_rate_LON,-300,300);
          GPS_I_LAT = GPS_I_LAT + (error_rate_LAT*Ki_gps*0.2);//5 Hz = 0.2
          GPS_I_LON = GPS_I_LON + (error_rate_LON*Ki_gps*0.2);  
          GPS_I_LAT = constrain(GPS_I_LAT,-150,150);//win speed +-200 cm/s
          GPS_I_LON = constrain(GPS_I_LON,-150,150);
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps + GPS_I_LAT;//PID Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps + GPS_I_LON;
          Control_XEf = constrain(Control_XEf,-800,800);//PWM 1000 - 1900
          Control_YEf = constrain(Control_YEf,-800,800);
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;//uAltitude = 1000 - 1900
          float upitchdesir = (Control_XEf*m_quad*-1.0)/uAltitude;//*-1
          urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          upitchdesir = constrain(upitchdesir,-0.7,0.7);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;
          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame
          //Control_XBf = constrain(Control_XBf, -20, 20);//+-20 deg
          //Control_YBf = constrain(Control_YBf, -20, 20);//+-20 deg
          
          //The desired roll and pitch angles by paper Modeling and Backstepping-based Nonlinear Control 
          //Ashfaq Ahman Mian 2008 (eq.25 and eq.26)
          //float urolldesir = ((Control_XEf*m_quad*sin(ahrs_y))/uAltitude) - ((Control_YEf*m_quad*cos_yaw)/uAltitude);
          //float upitchdesir = ((Control_XEf*m_quad)/(uAltitude*cos_roll*cos_yaw)) - ((sin(ahrs_r)*sin(ahrs_y))/(cos_roll*cos_yaw));
          //urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          //upitchdesir = constrain(upitchdesir,-0.7,0.7);
          //Control_YBf = asin(urolldesir)*RAD_TO_DEG;//Control roll eq.25 
          //Control_XBf = asin(upitchdesir)*RAD_TO_DEG*-1.0;//Control roll eq.26
          Control_XBf = constrain(Control_XBf, -20, 20);//+-20 +- 44
          Control_YBf = constrain(Control_YBf, -20, 20);
  }
void Cal_GPS(){
  //Apply moving average filter to GPS data
      GPS_filter_index = (GPS_filter_index+1) % 4;
      GPS_SUM_LAT[GPS_filter_index] = GPS_LAT1;
      GPS_SUM_LON[GPS_filter_index] = GPS_LON1;
   float sum1=0.0;
   float sum2=0.0;
  for(int i=0;i<4;i++)
  {
    sum1 += GPS_SUM_LAT[i];
    sum2 += GPS_SUM_LON[i];
  }
   GPS_LAT1f = sum1/4.0;
   GPS_LON1f = sum2/4.0;
////////////////////////////////////////
     //*Diff speed
     actual_speedX = (GPS_LAT1f - GPS_LAT1_old)/0.3;//cm/s  10000000.0 ,R = 6371000.0
     actual_speedY = (GPS_LON1f - GPS_LON1_old)/0.3;//cm/s
     //actual_speedX = constrain(actual_speedX, -400, 400);//+-400 cm/s
     //actual_speedY = constrain(actual_speedY, -400, 400);//+-400 cm/s  
     GPS_LAT1_old = GPS_LAT1f;
     GPS_LON1_old = GPS_LON1f;
     actual_speedXf = (actual_speedX + actual_speedXold)/2.0;//Moving Average Filters/
     actual_speedYf = (actual_speedY + actual_speedYold)/2.0;//Moving Average Filters/
     actual_speedXold = actual_speedX;
     actual_speedYold = actual_speedY;
     //*/
/////////////LeadFilter GPS/////////////////////////////////
    float lag_in_seconds = 0.85;//1.0 0.5
    float accel_contribution = (actual_speedXf - _last_velocityX) * lag_in_seconds * lag_in_seconds;
    float vel_contribution = actual_speedXf * lag_in_seconds;
    _last_velocityX = actual_speedXf;    // store velocity for next iteration
    GPS_LAT1lead = GPS_LAT1f  + vel_contribution + accel_contribution;
    float accel_contributio = (actual_speedYf - _last_velocityY) * lag_in_seconds * lag_in_seconds;
    float vel_contributio = actual_speedYf * lag_in_seconds;
    _last_velocityY = actual_speedYf;    // store velocity for next iteration
    GPS_LON1lead = GPS_LON1f  + vel_contributio + accel_contributio;
}
  
void Chack_Command(){
   if(AUX_1 <= (AltHold-10))//Stabilize 
  {
    Mode = 0;
  }
   if(AUX_1 > (AltHold-10) && AUX_1 <= (AltHold+10))//Altitude Hold, 
  {
    Mode = 2;//1
  }
   if(AUX_1 > (PositionHold-10) && AUX_1 <= (PositionHold+10))//Position Hold
  {
    Mode = 2;
  }  
   if(AUX_1 > (Auto-10))//Automatic  Takeoff
  {
    Mode = 3;
  }
///////////////////////////////////////////////
  //Chack_Command By  Remote  idle-up settings 0,1,2
          if(AUX_2 > 1750)//Set Waypoint 1 Quadrotor
          {
           waypoint1_LAT = GPS_LAT1f;
           waypoint1_LON = GPS_LON1f;
           digitalWrite(30, HIGH);//C
          }
          if(AUX_2 > 1250 && AUX_2 <= 1750)//Set HOME Quadrotor
          {
           GPS_LAT_HOME = GPS_LAT1f;
           GPS_LON_HOME = GPS_LON1f;
           //digitalWrite(30, LOW);
           digitalWrite(13, HIGH);//A
          }
//////////////////////////////////////////////
}
