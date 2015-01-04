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
//Channel
int CH_THR = 1000;
int CH_AIL = 1500;
int CH_ELE = 1500;
int CH_RUD = 1500;
float CH_AILf = 1500;
float CH_ELEf = 1500;
float CH_RUDf = 1500;
int CH_AIL_Cal = 1500;
int CH_ELE_Cal = 1500;
int CH_RUD_Cal = 1500;
int AUX_1 = 1500;
int AUX_2 = 1500;
int AUX_3 = 1500;
int AUX_4 = 1500;

volatile int16_t failsafeCnt = 0;
//
#define THROTTLEPIN                2 
#define ROLLPIN                    4 
#define PITCHPIN                   5  
#define YAWPIN                     6  
#define AUX1PIN                    7 
#define AUX2PIN                    0  
#define CAM1PIN                    1 
#define CAM2PIN                    3 

#define ROLL       0
#define PITCH      1
#define YAW        3
#define THROTTLE   2
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7      

static uint8_t pinRcChannel[8] = {ROLL, PITCH, THROTTLE, YAW, AUX1,AUX2,CAMPITCH,CAMROLL};
volatile uint16_t rcPinValue[8] = {1500,1500,1000,1500,1500,1500,1500,1500};
static int16_t rcData[8] ;
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];

// Read PPM SUM RX Data 
void rxInt() {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan1 = 0;  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan1 = 0;
    else {
      if(900<diff && diff<2200 && chan1<8) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcPinValue[chan1] = diff;
      }
    chan1++;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect) { if(PINK & (1<<0)) rxInt(); }//PPM_SUM at THROTTLE PIN on MEGA boards
// attach ISR to input trigger for PL1
//ISR(TIMER5_CAPT_vect) { rxInt(); } // bind rxInt() to capture event
///////////////////////////////////////////////////////////////////////////////////////////
void configureReceiver() {

    for (uint8_t chan = 0; chan < 8; chan++){
      for (uint8_t a = 0; a < 4; a++){
        rcData4Values[chan][a] = 1500;
      }  
    }    
//configure THROTTLE PIN (A8 pin) as input witch pullup and enabled PCINT interrupt
DDRK &= ~(1<<0); PORTK |= (1<<0); PCMSK2 |= (1<<0); PCICR |= (1<<2);
/*
// PPM_ON_PL1 pin PPM V2
// initialize timer 5 and trigger on rising flank
TCCR5A =((1<<WGM50)|(1<<WGM51));
TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5)); 
TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5)|(1<<ICNC5));//ICNC5 = Input Capture Noise Cancellation 5
OCR5A = 40000; // 0.5us per timertick
TIMSK5 |= (1<<ICIE5); // activate input capture for PL1
*/
}
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  data = rcPinValue[pinRcChannel[chan]];
  SREG = oldSREG;
  return data; 
}
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  rc4ValuesIndex++;
  for (chan = 0; chan < 8; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++){
      rcData[chan] += rcData4Values[chan][a];
    }
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -3)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +3)  rcHysteresis[chan] = rcData[chan]-2;
  }
    CH_THR = rcHysteresis[THROTTLE];
    CH_AIL = rcHysteresis[ROLL];
    CH_ELE = rcHysteresis[PITCH];
    CH_RUD = rcHysteresis[YAW];
    AUX_1 = rcHysteresis[AUX1];
    AUX_2 = rcHysteresis[AUX2];//rcPinValue[0]
    AUX_3 = rcHysteresis[CAMPITCH];
    AUX_4 = rcHysteresis[CAMROLL];
    CH_AILf = CH_AILf + (CH_AIL - CH_AILf)*0.02/tarremote;
    CH_ELEf = CH_ELEf + (CH_ELE - CH_ELEf)*0.02/tarremote;
    CH_RUDf = CH_RUDf + (CH_RUD - CH_RUDf)*0.02/tarremote;
}
void RC_Calibrate(){
  Serial.print("RC_Calibrate");Serial.println("\t");
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  CH_ELE_Cal = CH_ELE;
  CH_RUD_Cal = CH_RUD;
    Serial.print(CH_AIL_Cal);Serial.print("\t");//-0.13
    Serial.print(CH_ELE_Cal);Serial.print("\t");//-0.10
    Serial.print(CH_RUD_Cal);Serial.println("\t");//0.03 
}
