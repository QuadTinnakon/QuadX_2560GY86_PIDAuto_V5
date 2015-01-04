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
int32_t GPS_coord[2];// 1e-7 degrees / position as degrees (*10E7)
float GPS_ground_course = 0.0;
// velocities in cm/s if available from the GPS
int32_t _velocity_north;
int32_t _velocity_east;
int32_t _vel_down;

int LAT=0;
int LON=1;
uint8_t GPS_FIX=0;
int GPS_numSat=0;
int GPS_altitude=0;
int GPS_Present=0;
float Alt_Home = 2;//m
float yaw = 0.0;
float pitch = 0.0;
float Distance = 0.0;
#define ToRad 0.01745329252  // *pi/180
#define ToDeg 57.2957795131  // *180/pi
//prog_char
prog_char UBLOX_INIT[] PROGMEM = { // PROGMEM array must be outside any function !!!
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                //disable all default NMEA messages
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15, //0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
     0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
     0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A       // set rate to 5Hz
 };
 //http://www.multiwii.com/forum/viewtopic.php?f=6&t=4964
 //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
 //#define UBLOX_1HZ        0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39       // set rate to 1Hz
 // #define UBLOX_2HZ        0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77       // set rate to 2Hz
 //#define UBLOX_3HZ        0xB5,0x62,0x06,0x08,0x06,0x00,0x4D,0x01,0x01,0x00,0x01,0x00,0x64,0x8D       // set rate to 3Hz
 // #define UBLOX_4HZ        0xB5,0x62,0x06,0x08,0x06,0x00,0xFA,0x00,0x01,0x00,0x01,0x00,0x10,0x96       // set rate to 4Hz
 //#define UBLOX_5HZ        0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A       // set rate to 5Hz
 #define UBLOX_SET_BINARY "$PUBX,41,1,0003,0001,38400,0*26\n\265\142\006\001\003\000\001\006\001\022\117"
void GPS_multiInt()
{
  //Serial2.begin(115200);//u-blox-config.ublox.txt
  Serial2.begin(38400);//GPS2 38400
  //GPS UBLOX switch UART speed for sending SET BAUDRATE command (NMEA mode)
 // (which depends on the NMEA or MTK_BINARYxx settings)
  //Serial2.print("$PUBX,41,1,0003,0001,38400,0*26\r\n");
  //Serial2.print("$PUBX,41,1,0003,0001,57600,0*2D\r\n");
  delay(200);
  //Serial2.begin(57600);
  //Serial2.print("$PUBX,41,1,0003,0001,38400,0*26\r\n");
  //Serial2.print("$PUBX,41,1,0003,0001,57600,0*2D\r\n");
  delay(200);
  //Serial2.begin(115200);
  for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
    Serial2.write(pgm_read_byte(UBLOX_INIT+i));
    delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
   }
}
//(which depends on the NMEA or MTK_BINARYxx settings)

struct ubx_header {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  };
  struct ubx_nav_posllh {
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
  };
  struct ubx_nav_solution {
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
  };
  struct ubx_nav_velned {
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
  };
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
  static uint8_t _class;
  static uint8_t _fix_ok;
  // Receive buffer
  static union {
    ubx_nav_posllh posllh;
//    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[];
   } _buffer;
   
  bool UBLOX_parse_gps(void) {
    GPS_Present = 1;
    switch (_msg_id) {
    case MSG_POSLLH:
      //i2c_dataset.time                = _buffer.posllh.time;
      if(_fix_ok) {
        GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude   = _buffer.posllh.altitude_msl / 1000;      //alt in m
      }
      GPS_FIX = _fix_ok;
      return true;        // POSLLH message received, allow blink GUI icon and LED
      break;
    case MSG_SOL:
      _fix_ok = 0;
      if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) _fix_ok = 1;
      GPS_numSat = _buffer.solution.satellites;
      break;
    case MSG_VELNED:
      GPS_speed         = _buffer.velned.speed_2d;  // cm/s
      GPS_ground_course = (_buffer.velned.heading_2d / 100000.0); // (uint16_t) Heading 2D deg * 100000 rescaled to deg * 10
      _velocity_north  = _buffer.velned.ned_north;
      _velocity_east   = _buffer.velned.ned_east;
      _vel_down   = _buffer.velned.ned_down;
      break;
    default:
      break;
    }
    return false;
  }

  bool GPS_UBLOX_newFrame(uint8_t data){
    bool parsed = false;
    GPS_Present = 0;
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
        _class = data;
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
        //GPS_Present = 1;
        if (UBLOX_parse_gps()) { parsed = true;}
    } //end switch
    return parsed;
  }
