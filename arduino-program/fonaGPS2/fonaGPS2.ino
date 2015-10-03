/**
 *
 *
 */
#include <SPI.h>
//#include <SD.h>
#include <SdFat.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//#include "crc16.h"

#include <avr/pgmspace.h>

#define prog_char  char PROGMEM


// MPU-9150
const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MaX, MaY, MaZ;
char MPU9150_temp_str[6] = " 00.0";

uint16_t eeprom_index = 0;
// Seconds to wait before a new sensor reading is logged.
//#define LOGGING_FREQ_SECONDS   120
// data saved consumes 39bytes+81bytes per run, then add 1 byte at end.

// Number of times to sleep (for 8 seconds)
#define MAX_SLEEP_ITERATIONS_GPS   LOGGING_FREQ_SECONDS / 8
#define MAX_SLEEP_ITERATIONS_POST  MAX_SLEEP_ITERATIONS_GPS * 10

//#define OLD_DEBUG
#define DHT11_pin       15
#define FONA_RX         8
#define FONA_TX         9
#define FONA_RST        2
#define FONA_POWER_KEY  5
#define FONA_PSTAT      4
#define SDCARD_CS       10

uint8_t GPS_WAIT       = 0;
uint8_t GPS_AVG        = 0;
uint8_t GPS_FIX_MIN    = 0;
uint8_t NUMBER_OF_DATA = 0;
uint8_t LOGGING_FREQ_SECONDS = 0;
uint8_t DEBUG = 0;

uint8_t dataDHT11[6];
char DHT11_hum_str[3];
char DHT11_temp_str[3];

uint8_t samples = 1;
char dataBuffer[80];

char EEMEM data[26 + 45 * 10 + 12];
//char data[26+45*1+12] = {0};

// USED BY: sendDataServer loadConfigSDcard
//12345678901234567890123456789012345678901234567890
char url[50] = "0000000000000000000000000000000000000000000000000";
//char url[] ="http://cloud-mare.hummelgard.com:88/addData";

char NAME_str[20] = "0000000000000000000";
// USED BY: sendDataServer
char    IMEI_str[29] = "123456789012345";
//865067020395128
uint8_t  batt_state;
uint8_t  batt_percent;
uint16_t  batt_voltage;
char batt_volt_str[5] = "0000";
char batt_percent_str[4]="000";

// USED BY: loadConfigSDcard sendDataServer enableGprsFONA
char apn[30] = "00000000000000000000000000000";
char user[15] = "00000000000000";
char pwd[15] = "00000000000000";


// USED BY: readGpsFONA808
char latitude_str[10] = " 00.00000";
char longitude_str[10] = " 00.00000";

double lat;
double lon;
double latAVG;
double lonAVG;

char date_str[7] = "000000";
char time_str[7] = "000000";


#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_X        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R  
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R  
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_TEMP_OUT_H         0x41   // R  
#define MPU9150_TEMP_OUT_L         0x42   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R  
#define MPU9150_EXT_SENS_DATA_00   0x49   // R  
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R  
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R  
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R  
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R  
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R  
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R  
#define MPU9150_EXT_SENS_DATA_07   0x50   // R  
#define MPU9150_EXT_SENS_DATA_08   0x51   // R  
#define MPU9150_EXT_SENS_DATA_09   0x52   // R  
#define MPU9150_EXT_SENS_DATA_10   0x53   // R  
#define MPU9150_EXT_SENS_DATA_11   0x54   // R  
#define MPU9150_EXT_SENS_DATA_12   0x55   // R  
#define MPU9150_EXT_SENS_DATA_13   0x56   // R  
#define MPU9150_EXT_SENS_DATA_14   0x57   // R  
#define MPU9150_EXT_SENS_DATA_15   0x58   // R  
#define MPU9150_EXT_SENS_DATA_16   0x59   // R  
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R  
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R  
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R  
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R  
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R  
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R  
#define MPU9150_EXT_SENS_DATA_23   0x60   // R  
#define MPU9150_MOT_DETECT_STATUS  0x61   // R  
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x03   // R
#define MPU9150_CMPS_XOUT_H        0x04   // R
#define MPU9150_CMPS_YOUT_L        0x05   // R
#define MPU9150_CMPS_YOUT_H        0x06   // R
#define MPU9150_CMPS_ZOUT_L        0x07   // R
#define MPU9150_CMPS_ZOUT_H        0x08   // R

int MPU9150_I2C_ADDRESS = 0x68;

// This is to handle the absence of software serial on platforms
// like the Ardu = 0ino Due. Modify this code if you are using different
// hardware serial port, or if you are using a non-avr platform
// that supports software serial.
#ifdef __AVR__
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
SoftwareSerial serialLCD(6, 7);
#else
HardwareSerial *fonaSerial = &Serial1;
#endif

//Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t sleepIterations = MAX_SLEEP_ITERATIONS_GPS;
volatile bool watchdogActivated = true;


// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.

  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}


/***SERIAL DISPLAY **********************************************************/

void messageLCD(const int time, const String& line1, const String& line2 = "") {

  serialLCD.write(254); //clear display
  serialLCD.write(1);
  delay(10);
  serialLCD.write(124); //set brightness
  //serialLCD.write(129);  //min brightness
  //serialLCD.write(140);  //40% brightness
  serialLCD.write(157);  //max brightness
  delay(10);
  serialLCD.write(254);
  serialLCD.write(128);
  serialLCD.print(line1);
  serialLCD.write(254);
  serialLCD.write(192);
  serialLCD.print(line2);
  if (time > 0)
    delay(time);
  else if (time < 0) {
    delay(-time);
    serialLCD.write(254); //clear display
    serialLCD.write(1);
    serialLCD.write(124); //turn of backlight
    serialLCD.write(128);
    delay(10);
  }

}

uint32_t expectPulse(bool level) {

  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  uint8_t portState = level ? digitalPinToBitMask(DHT11_pin) : 0;
  while ((*portInputRegister(digitalPinToPort(DHT11_pin)) & digitalPinToBitMask(DHT11_pin)) == portState) {
    if (count++ >= microsecondsToClockCycles(1000)) {
      return 0; // Exceeded timeout, fail.
    }
  }

  return count;
}

boolean readDHT11() {
  bool _lastresult;
  // Reset 40 bits of received data to zero.
  dataDHT11[0] = dataDHT11[1] = dataDHT11[2] = dataDHT11[3] = dataDHT11[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.$

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(DHT11_pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(DHT11_pin, OUTPUT);
  digitalWrite(DHT11_pin, LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    //InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(DHT11_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.

    pinMode(DHT11_pin, INPUT);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i + 1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles  = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      _lastresult = false;
      return _lastresult;
    }
    dataDHT11[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      dataDHT11[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (dataDHT11[4] == ((dataDHT11[0] + dataDHT11[1] + dataDHT11[2] + dataDHT11[3]) & 0xFF)) {
    _lastresult = true;

    itoa(dataDHT11[0], DHT11_hum_str, 10);
    itoa(dataDHT11[2], DHT11_temp_str, 10);

    return _lastresult;
  }
  else {
    _lastresult = false;
    return _lastresult;
  }
}


int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H<<8)+L);
}


int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}

boolean initMPU9150() {

  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B00000010);     //Wake up MPU
  MPU9150_writeSensor(MPU9150_SMPLRT_DIV, B00000001);     //Set sample rate divider
  MPU9150_writeSensor(MPU9150_CONFIG, 6);                 //Set lowpass filter to 5Hz
  //MPU9150_writeSensor(MPU9150_GYRO_CONFIG, B00011000);    //Set gyro scale to +/-2000deg/s
  MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, B00000000);   //Set accelerometer range to +/-2g
  MPU9150_writeSensor(MPU9150_USER_CTRL, B00000000);      //Disable MPU as master for I2C slave
  MPU9150_writeSensor(MPU9150_INT_PIN_CFG, B00110010);    //Set bypass mode on I2C slave
  //MPU9150_writeSensor(MPU9150_INT_ENABLE, B00000001);     //Set data ready pin on
  MPU9150_writeSensor(MPU9150_PWR_MGMT_2, B00000111);     //Put xyz gyros to standby
  return true;
  
  }


boolean sleepMPU9150() {
  
  delay(10);
  MPU9150_I2C_ADDRESS = 0x0C; 
  MPU9150_writeSensor(0x0A, B00000000);
  MPU9150_I2C_ADDRESS = 0x68; 
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B01000000);
  return true;
  
  }

boolean readMPU9150() {
  
  delay(10);
  MPU9150_I2C_ADDRESS = 0x68;
  
  Tmp = MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H)/340+36.53;

  GyX = MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);
  GyY = MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H);
  GyZ = MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H);
  AcX = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
  AcY = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
  AcZ = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
  
  MPU9150_I2C_ADDRESS = 0x0c;
  
  MPU9150_writeSensor(0x0A, 0x02);
  delay(10);
  MaX = MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
  MaY = MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
  MaZ = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
  dtostrf(Tmp, 5, 1, MPU9150_temp_str);

}

/***LOW LEVEL AT FONA COMMANDS***********************************************/

uint8_t ATreadFONA(uint8_t multiline = 0, int timeout = 10000) {

  uint16_t replyidx = 0;

  while (timeout--) {
    if (replyidx >= 254) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (fonaSS.available()) {
      char c =  fonaSS.read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (replyidx == 0)   // the first 0x0A is ignored
          continue;

        if (!multiline--) {
          timeout = 0;         // the second 0x0A is the end of the line
          break;
        }


      }
      if (c == 0xA)
        dataBuffer[replyidx] = ';';
      else
        dataBuffer[replyidx] = c;
      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);
      replyidx++;

    }

    //Serial.println(timeout);
    delay(1);
  }

  dataBuffer[replyidx] = 0;  // null term
  Serial.print(F("READ: "));
  Serial.println(dataBuffer);
  //delay(1000);
  return replyidx;
}


void ATsendFONA(char* ATstring) {

  //messageLCD(2000, String(ATstring));
  Serial.print(F("SEND: "));
  Serial.println(String(ATstring));

  fonaSS.println(String(ATstring));
  return;
}

uint8_t ATsendReadFONA(char* ATstring, uint8_t multiline = 0, int timeout = 10000) {

  //messageLCD(2000, String(ATstring));
  Serial.print(F("SEND: "));
  Serial.println(String(ATstring));

  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}

uint8_t ATsendReadFONA(const __FlashStringHelper *ATstring, uint8_t multiline = 0, int timeout = 10000) {

  //messageLCD(2000, String(ATstring));
  Serial.print(F("SEND: "));
  Serial.println(String(ATstring));

  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}


boolean ATsendReadVerifyFONA(char* ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0, int timeout = 10000) {

  if (ATsendReadFONA(ATstring, multiline, timeout)) {
    if ( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}


boolean ATsendReadVerifyFONA(const __FlashStringHelper *ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0, int timeout = 10000) {
  if (ATsendReadFONA(ATstring, multiline, timeout)) {
    if ( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}


void getImeiFONA() {

  ATsendReadFONA(F("AT+GSN"), 2);
  char* tok = strtok(dataBuffer, ";");
  strcpy(IMEI_str, tok);
}


uint8_t batteryCheckFONA() {

  ATsendReadFONA(F("AT+CBC"), 2);

  // typical string from FONA: "+CBC: 0,82,4057;OK"
  char* tok = strtok(dataBuffer, ":");
  tok = strtok(NULL, ",");
  batt_state = atoi(tok);

  tok = strtok(NULL, ",");
  batt_percent = atoi(tok);
  strcpy(batt_percent_str, tok);

  tok = strtok(NULL, ";");
  batt_voltage = atoi(tok);
  strcpy(batt_volt_str, tok);

  return batt_percent;
}

/***GPRS COMMANDS************************************************************/
boolean loadConfigSDcard() {
  File SDfile;
  SdFat SD;
  SD.begin(SDCARD_CS);
  SDfile = SD.open("config.txt");
  if (SDfile) {

    while (SDfile.available()) {

      // read one line at a time
      int index = 0;
      do {
        //while( dataBuffer[index] !='\n' ){
        dataBuffer[index] = SDfile.read();
      }
      while (dataBuffer[index++] != '\n');

      if ( dataBuffer[0] != '#') {

        char* parameter = strtok(dataBuffer, "=");
        char* value = strtok(NULL, "\n");

        if (! strcmp_P(value, (const char PROGMEM *)F("VOID")) == 0 ) {


          if ( strcmp_P(parameter, (const char PROGMEM *)F("apn")) == 0 )
            strcpy(apn, value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("user")) == 0 )
            strcpy(user, value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("pwd")) == 0 )
            strcpy(pwd, value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("url")) == 0 )
            strcpy(url, value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("name")) == 0 )
            strcpy(NAME_str, value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
            LOGGING_FREQ_SECONDS = atoi(value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("GPS_WAIT")) == 0 )
            GPS_WAIT = atoi(value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
            GPS_FIX_MIN = atoi(value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("GPS_AVG")) == 0 )
            GPS_AVG = atoi(value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
            NUMBER_OF_DATA = atoi(value);

          if ( strcmp_P(parameter, (const char PROGMEM *)F("DEBUG")) == 0 )
            DEBUG = atoi(value);

          Serial.print("SD: ");
          Serial.print(parameter);
          Serial.print("=");
          Serial.println(value);
        }
      }
    }
    SDfile.close();
  }
  return true;
}


boolean disableGprsFONA() {

  if (! ATsendReadVerifyFONA(F("AT+CIPSHUT"), F("SHUT OK")) )
    return false;

  if (! ATsendReadVerifyFONA(F("AT+CGATT?"), F("+CGATT: 0;;OK"), 2) )
    return false;
}


boolean enableGprsFONA() {


  if (! ATsendReadVerifyFONA(F("AT+CIPSHUT"), F("SHUT OK")) )
    return false;

  if ( ATsendReadVerifyFONA(F("AT+CGATT?"), F("+CGATT: 1;;OK"), 2) ) {

    if (! ATsendReadVerifyFONA(F("AT+CGATT=0"), F("OK")) )
      return false;
  }
  else {
  }

  if (! ATsendReadVerifyFONA(F("AT+CGATT=1"), F("OK")) )
    return false;

  if (! ATsendReadVerifyFONA(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), F("OK")) )
    return false;

  strcpy(dataBuffer, "AT+SAPBR=3,1,\"APN\",\"");

  strcat(dataBuffer, apn);
  strcat(dataBuffer, "\"");
  if (! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
    return false;

  if (user == "") {
    strcpy(dataBuffer, "AT+SAPBR=3,1,\"USER\",\"");
    strcat(dataBuffer, user);
    strcat(dataBuffer, "\"");
    if (! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
      return false;
  }

  if (pwd == "") {
    strcpy(dataBuffer, "AT+SAPBR=3,1,\"PWD\",\"");
    strcat(dataBuffer, pwd);
    strcat(dataBuffer, "\"");
    if (! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
      return false;
  }

  if (! ATsendReadVerifyFONA(F("AT+SAPBR=1,1"), F("OK")) )
    return false;


  return true;
}

boolean initFONA() {

  fonaSS.begin(4800);
  fonaSS.flush();

  // Check if FONA is ON, if not turn it on!
  if (digitalRead(FONA_PSTAT) == false ) {
    pinMode(FONA_POWER_KEY, OUTPUT);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(100);
    digitalWrite(FONA_POWER_KEY, LOW);
    delay(2000);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(7000);

  }

  boolean reset = false;

  do {
    if ( reset == true ) {

      pinMode(FONA_RST, OUTPUT);
      digitalWrite(FONA_RST, HIGH);
      delay(10);
      digitalWrite(FONA_RST, LOW);
      delay(100);
      digitalWrite(FONA_RST, HIGH);
      delay(7000);
      fonaSS.flush();
      reset = false;
    }

    if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;
    delay(100);
    if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;
    delay(100);
    if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;
    delay(100);
    //turn off Echo!
    ATsendReadVerifyFONA(F("ATE0"), F("OK"));
    delay(100);

    //if(! ATsendReadVerifyFONA(F("ATE0"), F("OK")) )
    //  reset = true;

    // turn on hangupitude
    //if(! ATsendReadVerifyFONA(F("AT+CVHU=0"), F("OK")) )
    //  reset = true;
    //delay(100);
  } while (reset == true);

  return true;
}


boolean enableGpsFONA808(void) {

  // first check if GPS is already on or off
  if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;;OK"), 2) ) {
    return true;
  }
  else {
    if (! ATsendReadVerifyFONA(F("AT+CGPSPWR=1"), F("OK")) )
      return false;
  }

  return true;
}

uint8_t readGpsFONA808() {

  //READ: +CGPSINF: 32,061128.000,A,6209.9268,N,01710.7044,E,0.000,292.91,110915,;

  uint8_t timeout = GPS_WAIT;
  uint8_t fix_status;
  while (timeout--) {

    if ( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("+CGPSSTATUS: Location Not Fix;;OK"), 2) )
      fix_status = 1;
    else if (dataBuffer[22] == '3')
      fix_status = 3;
    else if (dataBuffer[22] == '2')
      fix_status = 2;
    else if (dataBuffer[22] == 'U')
      fix_status = 0;

    if (fix_status >= GPS_FIX_MIN) {

      ATsendReadFONA(F("AT+CGPSINF=32"), 2);
      //strcpy(dataBuffer,"+CGPSINF: 32,061128.000,A,6209.9268,N,01710.7044,E,0.000,292.91,110915,");

      //-------------------------------
      // skip mode
      char *tok = strtok(dataBuffer, ",");
      if (! tok) return false;

      // grab current UTC time hhmmss.sss ,-skip the last three digits.
      tok = strtok(NULL, ",");
      if (! tok) return false;
      else strncpy(time_str, tok, 6);

      // check fix
      tok = strtok(NULL, ",");
      if (! tok) return false;

      if (!tok[0] == 'xV')
        return false;


      // grab the latitude
      char *latp = strtok(NULL, ",");
      if (! latp) return false;

      // grab latitude direction
      char *latdir = strtok(NULL, ",");
      if (! latdir) return false;

      // grab longitude
      char *longp = strtok(NULL, ",");
      if (! longp) return false;

      // grab longitude direction
      char *longdir = strtok(NULL, ",");
      if (! longdir) return false;

      // skip speed
      tok = strtok(NULL, ",");
      if (! tok) return false;

      // skip course
      tok = strtok(NULL, ",");
      if (! tok) return false;

      // grab date ddmmyy
      tok = strtok(NULL, ",");
      if (! tok) return false;
      else strcpy(date_str, tok);

      double latitude = atof(latp);
      double longitude = atof(longp);

      // convert latitude from minutes to decimal
      double degrees = floor(latitude / 100);
      double minutes = latitude - (100 * degrees);
      minutes /= 60;
      degrees += minutes;

      // turn direction into + or -
      if (latdir[0] == 'S') degrees *= -1;

      dtostrf(degrees, 9, 5, latitude_str);
      lat = degrees;

      // convert longitude from minutes to decimal
      degrees = floor(longitude / 100);
      minutes = longitude - (100 * degrees);
      minutes /= 60;
      degrees += minutes;

      // turn direction into + or -
      if (longdir[0] == 'W') degrees *= -1;

      dtostrf(degrees, 9, 5, longitude_str);
      lon = degrees;
      //-------------------------
      //delay(2000);
      return fix_status;
    }
    delay(2000);

  }

  return 0;
}


boolean powerOffFONA(boolean powerOffGPS = false) {

  if ( powerOffGPS ) {
    // first check if GPS is already on or off
    if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;;OK"), 2) ) {

      if (! ATsendReadVerifyFONA(F("AT+CGPSPWR=0"), F("OK")) )
        return false;
    }

  }
  if (! ATsendReadVerifyFONA(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 0) )
    return false;

  /*
  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == HIGH;
  delay(500);
  digitalWrite(FONA_POWER_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_POWER_KEY, HIGH);
  pinMode(FONA_POWER_KEY, OUTPUT);
  delay(500);
  */
  return true;
}


void clearInitData() {

  char str1[6] = "IMEI=";

  eeprom_index = 0;

  eeprom_write_block(str1, &data[eeprom_index], 5);
  eeprom_index += 5;

  eeprom_write_block(IMEI_str, &data[eeprom_index], strlen(IMEI_str));
  eeprom_index += strlen(IMEI_str);

  char str2[7] = "&name=";

  eeprom_write_block(str2, &data[eeprom_index], 6);
  eeprom_index += 6;

  eeprom_write_block(NAME_str, &data[eeprom_index], strlen(NAME_str));
  eeprom_index += strlen(NAME_str);


  char str3[7] = "&data=";

  eeprom_write_block(str3, &data[eeprom_index], 6);
  eeprom_index += 6;



}


void saveData() {

  char square[] = "#";

  eeprom_write_block(latitude_str, &data[eeprom_index], 9);
  eeprom_index += 9;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(longitude_str, &data[eeprom_index], 9);
  eeprom_index += 9;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(date_str, &data[eeprom_index], 6);
  eeprom_index += 6;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(time_str, &data[eeprom_index], 6);
  eeprom_index += 6;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(batt_volt_str, &data[eeprom_index], 4);
  eeprom_index += 4;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;
  
  eeprom_write_block(batt_percent_str, &data[eeprom_index], strlen(batt_percent_str));
  eeprom_index += strlen(batt_percent_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;
  
  eeprom_write_block(DHT11_hum_str, &data[eeprom_index], 2);
  eeprom_index += 2;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(DHT11_temp_str, &data[eeprom_index], 2);
  eeprom_index += 2;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(MPU9150_temp_str, &data[eeprom_index], 5);
  eeprom_index += 5;

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  char AcX_str[8] = "0000000";
  char AcY_str[8] = "0000000";
  char AcZ_str[8] = "0000000";

  itoa(AcX, AcX_str, 10);
  itoa(AcY, AcY_str, 10);
  itoa(AcZ, AcZ_str, 10);

  eeprom_write_block(AcX_str, &data[eeprom_index], strlen(AcX_str));
  eeprom_index += strlen(AcX_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(AcY_str, &data[eeprom_index], strlen(AcY_str));
  eeprom_index += strlen(AcY_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(AcZ_str, &data[eeprom_index], strlen(AcZ_str));
  eeprom_index += strlen(AcZ_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  char MaX_str[6] = "00000";
  char MaY_str[6] = "00000";
  char MaZ_str[6] = "00000";

  itoa(MaX, MaX_str, 10);
  itoa(MaY, MaY_str, 10);
  itoa(MaZ, MaZ_str, 10);

  eeprom_write_block(MaX_str, &data[eeprom_index], strlen(MaX_str));
  eeprom_index += strlen(MaX_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(MaY_str, &data[eeprom_index], strlen(MaY_str));
  eeprom_index += strlen(MaY_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;

  eeprom_write_block(MaZ_str, &data[eeprom_index], strlen(MaZ_str));
  eeprom_index += strlen(MaZ_str);

  eeprom_write_block(square, &data[eeprom_index], 1);
  eeprom_index += 1;
}


uint16_t sendDataServer(char* error_code) {

  // close all prevoius HTTP sessions
  ATsendReadVerifyFONA(F("AT+HTTPTERM"), F("OK"), 0);

  // start a new HTTP session
  if (! ATsendReadVerifyFONA(F("AT+HTTPINIT"), F("OK")) )
    return false;

  // setup the HTML HEADER
  // CID = Bearer profile identifier =
  if (! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"CID\",\"1\""), F("OK")) )
    return false;

  // setup the HTML USER AGENT
  if (! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"UA\",\"CLOUDMARE1.0\""), F("OK")) )
    return false;

  // setup the HTML CONTENT
  if (! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""), F("OK")) )
    return false;

  // setup URL to send data to
  strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPPARA=\"URL\",\""));
  strcat(dataBuffer, url);
  strcat(dataBuffer, "\"");
  if (! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
    return false;

  // crop of the last "#" in data string
  eeprom_index--;

  // add final parameter, -the bit-checksum
  char str1[] = "&sum=";
  eeprom_write_block(str1, &data[eeprom_index], 5);
  eeprom_index += 5;


  uint16_t sum = 0;
  char pop[1];
  for (uint16_t i = 0; i < eeprom_index; i++) {
    eeprom_read_block(pop, &data[i], 1);
    sum += __builtin_popcount(pop[0]);
  }
  char sum_str[6];

  itoa(sum, sum_str, 10);
  strcat(sum_str, "&");

  eeprom_write_block(sum_str, &data[eeprom_index], strlen(sum_str));
  eeprom_index += strlen(sum_str);


  for (uint16_t i = 0; i < eeprom_index; i++) {
    eeprom_read_block(pop, &data[i], 1);
    Serial.write(*pop);
    //if((i-5)%40 == 0)
  }
  Serial.write('\n');

  // setup length of data to send
  strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));
  char dataLengthStr[4];

  itoa(eeprom_index + 1, dataLengthStr, 10);
  strcat(dataBuffer, dataLengthStr);

  strcat(dataBuffer, ",");
  strcat_P(dataBuffer, (const char PROGMEM *)F("10000"));
  if (! ATsendReadVerifyFONA(dataBuffer, F("DOWNLOAD"), 0) )
    return false;

  // downloading data to send to FONA
  for ( uint16_t i = 0; i < eeprom_index; i++) {
    eeprom_read_block(pop, &data[i], 1);
    fonaSS.write(pop[0]);
    Serial.write(pop[0]);
  }
  fonaSS.write('\n');
  Serial.write('\n');

  // Check if download was OK? (-eat up OK)
  ATreadFONA();


  // sending data by HTTP POST
  if (! ATsendReadVerifyFONA(F("AT+HTTPACTION=1"), F("OK"), 0) )
    return false;

  // read reply from server, HTTP code
  ATreadFONA(0, 11000);
  char* code = strtok(dataBuffer, ",");
  code = strtok(NULL, ",");
  strcpy(error_code, code);

  if ( atoi(code) == 200 || atoi(code) == 302 )
    return true;
  else
    return false;
}


//---------------------------------------
/* SKRIV TILL FLASH MINNE
char EEMEM eepromString[10]; //declare the flsah memory.

    while ( !eeprom_is_ready());
    char save[] ="bananpaj";
    cli();    //disable interupts so it's not disturbed during write/read
    eeprom_write_block(save, &eepromString[5],sizeof(save));
     while ( !eeprom_is_ready());
     char ramString[10];
     eeprom_read_block( &ramString[0], &eepromString[5], sizeof(save));
      sei(); //enable itnerupts again.
     Serial.println(ramString);
     */
//---------------------------------------


//SETUP
//-------------------------------------------------------------------------------------------
void setup() {

  // MPU-9050 9-deg accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);



  //DHT11 temperature sensor digital pin
  pinMode(DHT11_pin, INPUT);
  digitalWrite(DHT11_pin, HIGH);

  serialLCD.begin(9600);
  delay(500);
  Serial.begin(115200);

  pinMode(FONA_PSTAT, INPUT);
  pinMode(FONA_POWER_KEY, OUTPUT);
  digitalWrite(FONA_POWER_KEY, HIGH);

  // This next section of code is timing critical, so interrupts are disabled.
  noInterrupts();

  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1 << WDRF);

  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1 << WDP0) | (1 << WDP3);

  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1 << WDIE);

  // Enable interrupts again.
  interrupts();

}

//LOOP
//-------------------------------------------------------------------------------------------
void loop() {
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated) {
    watchdogActivated = false;

    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;

    if (sleepIterations >= MAX_SLEEP_ITERATIONS_GPS) {

      //AWAKE, -DO SOME WORK!
      messageLCD(1000, F("ARDUINO"), F(">booting"));


      // Reset the number of sleep iterations.
      sleepIterations = 0;


      // READ DATA FROM TEMP/HUMID SENSOR DHT11
      readDHT11();


      // READ DATA FROM ACCELEROMETER MPU9150
      initMPU9150();
      //delay(100);
      readMPU9150();
      /*
      messageLCD(500, "MaX:", String(MaX));
      messageLCD(500, "MaY:", String(MaY));
      messageLCD(500, "MaZ:", String(MaZ));
      messageLCD(500, "AcX:", String(AcX));
      messageLCD(500, "AcY:", String(AcY));
      messageLCD(500, "AcZ:", String(AcZ));
      messageLCD(500, "GyX:", String(GyX));
      messageLCD(500, "GyY:", String(GyY));
      messageLCD(500, "GyZ:", String(GyZ));
      messageLCD(500, "TMP:", String(Tmp));
      */
      //delay(100);
      sleepMPU9150();
      //delay(5000);
      // START UP FONA MODULE
      //messageLCD(0, F("FONA:"), F(">on"));
      initFONA();


      // READ IMEI NUMBER OF SIM CARD
      getImeiFONA();
      //messageLCD(0, F("FONA imei:"), IMEI_str);


      // LOAD USER CONFIGUARTION FROM SDCARD
      loadConfigSDcard();


      // IS THIS FIRST RUN?, -THEN INIT/CLEAR EEPROM STORAGE
      if (samples == 1)
        clearInitData();


      // TURN ON THE GPS UNIT IN FONA MODULE
      //messageLCD(0, F("FONA-gps"), F(">on"));
      enableGpsFONA808();


      // CHECK BATTERY LEVEL
      delay(100);
      batteryCheckFONA();
      messageLCD(1000, "Battery %", String(batt_percent) );


      // TAKE GPS_AVG of GPS's READING FOLLOWED by AVERAGING
      lonAVG = 0;
      latAVG = 0;
      double lat1 = 0;
      double lat2 = 0;
      double lat3 = 0;
      double lat4 = 0;
      double lat5 = 0;
      double lat6 = 0;
      double lon1 = 0;
      double lon2 = 0;
      double lon3 = 0;
      double lon4 = 0;
      double lon5 = 0;
      double lon6 = 0;
      for (int i = 1; i <= GPS_AVG; i) {

        messageLCD(0, F("FONA-gps"), ">get #" + String(i) );

        if (readGpsFONA808()) {

          if ( lat > lat1 )
            if ( lat > lat2)
              if ( lat > lat3)
                if ( lat > lat4)
                  if ( lat > lat5)
                    if ( lat > lat6) {
                      lat1 = lat2;
                      lat2 = lat3;
                      lat3 = lat4;
                      lat4 = lat5;
                      lat5 = lat6;
                      lat6 = lat;
                    }
                    else {
                      lat1 = lat2;
                      lat2 = lat3;
                      lat3 = lat4;
                      lat4 = lat5;
                      lat5 = lat;
                    }
                  else {
                    lat1 = lat2;
                    lat2 = lat3;
                    lat3 = lat4;
                    lat4 = lat;
                  }
                else {
                  lat6 = lat5;
                  lat5 = lat4;
                  lat4 = lat;       
                }
              else {
                lat6 = lat5;
                lat5 = lat4;
                lat4 = lat3;                         
                lat3 = lat;
              }
            else {
              lat6 = lat5;
              lat5 = lat4;
              lat4 = lat3;                         
              lat2 = lat;             
            }
          else {
            lat6 = lat5;
            lat5 = lat4;
            lat4 = lat3;
            lat3 = lat2;
            lat2 = lat1;
            lat1 = lat;
          }

          if ( lon > lon1 )
            if ( lon > lon2)
              if ( lon > lon3)
                if ( lon > lon4)
                  if ( lon > lon5)
                    if ( lon > lon6) {
                      lon1 = lon2;
                      lon2 = lon3;
                      lon3 = lon4;
                      lon4 = lon5;
                      lon5 = lon6;
                      lon6 = lon;
                    }
                    else {
                      lon1 = lon2;
                      lon2 = lon3;
                      lon3 = lon4;
                      lon4 = lon5;
                      lon5 = lon;
                    }
                  else {
                    lon1 = lon2;
                    lon2 = lon3;
                    lon3 = lon4;
                    lon4 = lon;
                  }
                else {
                  lon6 = lon5;
                  lon5 = lon4;
                  lon4 = lon;
                }
              else {
                lon6 = lon5;
                lon5 = lon4;
                lon4 = lon3;                         
                lon3 = lon;
              }
            else {
              lon6 = lon5;
              lon5 = lon4;
              lon4 = lon3;                         
              lon2 = lon;             
            }
          else {
            lon6 = lon5;
            lon5 = lon4;
            lon4 = lon3;
            lon3 = lon2;
            lon2 = lon1;
            lon1 = lon;
          }

          //latAVG += lat;
          //lonAVG += lon;
          delay(2000);
          i++;
        }
      }
      //Serial.print(lat1,5);Serial.print(" ");Serial.print(lat2,5);Serial.print(" ");Serial.print(lat3,5);
      //Serial.print(" ");Serial.print(lat4,5);Serial.print(" ");Serial.println(lat5,5);
      latAVG = (lat3 + lat4)/2;
      lonAVG = (lon3 + lon4)/2;
      //latAVG/=GPS_AVG;
      //lonAVG/=GPS_AVG;

      dtostrf(latAVG, 9, 5, latitude_str);
      dtostrf(lonAVG, 9, 5, longitude_str);

/*
      char* line1_pointer = dataBuffer;
      char* line2_pointer = dataBuffer + 16;

      strcpy(dataBuffer, latitude_str);
      strcat(dataBuffer, " ");
      strcat(dataBuffer, date_str);
      strcat(dataBuffer, longitude_str);
      strcat(dataBuffer, " ");
      strcat(dataBuffer, time_str);
      messageLCD(4000, line1_pointer, line2_pointer );
*/


      // SAVING DATA TO EEPROM
      saveData();


      // TIME TO SEND DATA TO SERVER?
      if ( samples >= NUMBER_OF_DATA ) {


        // TURN ON GPRS
        enableGprsFONA();
        messageLCD(0, F("FONA-gprs"), F(">on"));


        // SENDING DATA BY HTTP POST
        char error_code[4] = "000";
        if ( sendDataServer(error_code) )
          messageLCD(2000, F("HTTP"), ">OK #" + String(error_code));
        else
          messageLCD(2000, F("HTTP"), ">ERR #" + String(error_code));

        samples = 0;

        // TURN OFF GPRS
        disableGprsFONA();
      }
      samples++;


      // power down FONA, true=GPS aswell
      powerOffFONA(true);
      //messageLCD(0, F("FONA"), F(">off"));


      // GO TO SLEEP
      messageLCD(-1000, F("ARDUINO"), F(">sleep"));
    }
  }

  sleep();

}

