/*
project_QuadX_2560 GY86_PIDAuto_V1  
1. Automatic  Takeoff 
2. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

 #include "Ultrasonic.h"
 Ultrasonic LV-EZ4  AnalogRead
 every 50mS, (20-Hz rate)
 from 6-inches out to 254-inches
 from 0.15-m out to 6.45 m
 5V yields ~9.8mV/in. , 1 in = 0.0254 m =~9.8mV/0.0254 m = 0.38582 v/m
 
 
 Ultrasonic_HC-SR04
  from 0.02 -m out to 1.4 m
 //Just connect VCC to (+) on D9, trig to D9, echo to D10, Gnd to (-)
 */
float hz_Ultra = 0.0;
float Altitude_sona = 0.0;
float Altitude_sona2 = 0.0;
float Altitude_sonaf = 0.0;
float Altitude_sonaold = 0.0;
float vz_sona = 0.0;
float vz_sona2 = 0.0;
float vz_sonaf = 0.0;

float Altitude_II = 0.0;
float Altitude_Baro_ult = 0.0;

#define HCSR04_TriggerPin 9 // should be modified to 9  12 in next version
#define HCSR04_EchoPin 10     // should be modified to 10  11 in next version
volatile unsigned long HCSR04_startTime = 0;
volatile unsigned long HCSR04_echoTime = 0;
volatile static int32_t  tempSonarAlt=0;

void UltrasonicInt()
{
  pinMode(HCSR04_EchoPin,INPUT);
  pinMode(HCSR04_TriggerPin,OUTPUT);
  PCMSK0 = (1<<PCINT4); // Mask Pin PCINT5 - all other PIns PCINT0-7 are not allowed to create interrupts!
  PCICR |= (1<<PCIE0); // PCINT 0-7 belong to PCIE0 //HCSR04_EchoPin_PCICR
}
// EchoPin will change to high signalize beginning
// and back to low after 58*cm us
// First interrupt is needed to start measurement, second interrupt to calculate the distance


ISR(PCINT0_vect) {
  //
  // Here is a routine missing, to check, if the interrupt was raised for echo pin - not needed at the moment, because we don't have any interrupts
  // for this interrupt group, but maybe later
  //
  if (PINB & (1<<PCINT4)) { //indicates if the EchoPin is at a high state
    HCSR04_startTime = micros();
  }
  else {
    HCSR04_echoTime = micros() - HCSR04_startTime;
    if (HCSR04_echoTime <= 25000)      // maximum = 4,31 meter - 30000 us means out of range
      tempSonarAlt = HCSR04_echoTime / 5.8;//to mm
    else
      tempSonarAlt = 4300;// max 4.3 m
  }
}

void UltrasonicRead()
{
  //Ultrasonic LV-EZ4
  //int sensorValue = analogRead(A0);
  //hz_Ultra = (sensorValue*5/1024.0)/0.38528;//5V yields ~9.8mV/in. , 1 in = 0.0254 m =~9.8mV/0.0254 m = 0.38582 v/m
  //hz_Ultra = constrain(hz_Ultra, 0, 6.45);//m
  
  //Ultrasonic_HC-SR04 // create a trigger pulse for 10 us
  digitalWrite(HCSR04_TriggerPin, LOW);
  //delayMicroseconds(2);
  digitalWrite(HCSR04_TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TriggerPin, LOW);
  Altitude_sona = tempSonarAlt/1000.0;//m
  //Altitude_sonaf = (Altitude_sona + Altitude_sona2)/2.0;//filter
  Altitude_sonaf = Altitude_sonaf + (((Altitude_sona + Altitude_sona2)/2.0) - Altitude_sonaf)*0.687;//filter 42 Hz
  Altitude_sona2 = Altitude_sona;
  vz_sona = (Altitude_sonaf - Altitude_sonaold)/0.1;//diff 
  //vz_sonaf = (vz_sona + vz_sona2)/2.0;//filter
  vz_sonaf = vz_sonaf + (((vz_sona + vz_sona2)/2.0) - vz_sonaf)*0.687;//filter 42 Hz
  vz_sona2 = vz_sona;
  vz_sonaf = constrain(vz_sonaf, -3, 3);
  //Altitude_sonano = constrain(Altitude_sonano, 0, 3.0);//m
  //Ultrasonic max 0.8 m change to Baro//////////////////////////////
if(Altitude_hat > 0.8){
  Altitude_Baro_ult = Altitude_barof;
}
else{
  float error_Altitude = Altitude_sonaf - Altitude_barof;
  Altitude_II = Altitude_II + (error_Altitude*0.0095);//0.005 ,,20 Hz = 0.05
  Altitude_Baro_ult = Altitude_sonaf;
}
////////////////////////////////////////////////////////////////
}
