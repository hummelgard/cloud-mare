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

<<<<<<< HEAD
// Number of times to sleep (for 8 seconds)
=======
>>>>>>> memory-otimization
#define MAX_SLEEP_ITERATIONS_GPS   LOGGING_FREQ_SECONDS / 8
#define MAX_SLEEP_ITERATIONS_POST  MAX_SLEEP_ITERATIONS_GPS * 10

#define DHT11_pin       15
#define FONA_RX         8
#define FONA_TX         9
#define FONA_RST        2
#define FONA_POWER_KEY  5
#define FONA_PSTAT      4
#define GPS_WAIT        200
#define SDCARD_CS       10
#define POS_SIZE        11


// MPU-9150 registers
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_TEMP_OUT_H         0x41   // R  
#define MPU9150_TEMP_OUT_L         0x42   // R 
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_CMPS_XOUT_L        0x03   // R
#define MPU9150_CMPS_XOUT_H        0x04   // R
#define MPU9150_CMPS_YOUT_L        0x05   // R
#define MPU9150_CMPS_YOUT_H        0x06   // R
#define MPU9150_CMPS_ZOUT_L        0x07   // R
#define MPU9150_CMPS_ZOUT_H        0x08   // R


uint8_t GPS_AVG        = 0;
uint8_t GPS_FIX_MIN    = 0;
uint8_t NUMBER_OF_DATA = 0;
uint8_t LOGGING_FREQ_SECONDS = 0;

<<<<<<< HEAD
// MPU-9150
const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MaX, MaY, MaZ;
char MPU9150_temp_str[6] = " 00.0";

uint16_t eeprom_index = 0;
// Seconds to wait before a new sensor reading is logged.
//#define LOGGING_FREQ_SECONDS   120
// data saved consumes 39bytes+81bytes per run, then add 1 byte at end.

uint8_t dataDHT11[6];
char DHT11_hum_str[3];
char DHT11_temp_str[3];

uint8_t samples = 1;
=======
uint8_t samples = 1;
uint8_t dataDHT11[6];
int16_t int16_i;
uint16_t eeprom_index = 0;
char str10_A[10];
char str10_B[10];
char str10_C[10];

>>>>>>> memory-otimization
char dataBuffer[80];

char EEMEM data[26 + 45 * 10 + 12];

// USED BY: sendDataServer loadConfigSDcard
//             12345678901234567890123456789012345678901234567890
//char url[] ="http://cloud-mare.hummelgard.com:88/addData";
char url[50] = "0000000000000000000000000000000000000000000000000";

<<<<<<< HEAD
// USED BY: sendDataServer
char NAME_str[20] = "0000000000000000000";
char    IMEI_str[29] = "123456789012345";

uint8_t  batt_state;
uint8_t  batt_percent;
uint16_t  batt_voltage;
char batt_volt_str[5] = "0000";
char batt_percent_str[4]="000";
=======
>>>>>>> memory-otimization

// USED BY: loadConfigSDcard sendDataServer enableGprsFONA
char apn[30]  = "00000000000000000000000000000";
char user[15] = "00000000000000";
char pwd[15]  = "00000000000000";


<<<<<<< HEAD
// USED BY: readGpsFONA808
char latitude_str[10] = " 00.00000";
char longitude_str[10] = " 00.00000";

double lat;
double lon;
double latAVG;
double lonAVG;

double latArray[POS_SIZE]={0};
double lonArray[POS_SIZE]={0};

char date_str[7] = "000000";
char time_str[7] = "000000";

char* char_pt1;
char* char_pt2;
char* char_pt3;
char* char_pt4;
char char_c;
byte byte_i;
byte byte_j;
uint8_t uint8_i;
uint8_t uint8_j;
uint8_t uint8_k;
int int_i;
uint16_t uint16_i;
uint16_t uint16_j;
uint32_t uint32_i;
uint32_t uint32_j;
float float_f;
double double_i;
double double_j;
double double_k;
double double_l;
char str1[6] = "IMEI=";
char str2[7] = "&name=";
char str3[7] = "&data=";
//char square1[] = "#";
char strA_8[8] = "0000000";
char str4[6] = "&sum=";
char pop[1];

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

=======
float lat;
float lon;
//float latAVG;
//float lonAVG;
float latArray[POS_SIZE]={0};
float lonArray[POS_SIZE]={0};
uint8_t sleepIterations = MAX_SLEEP_ITERATIONS_GPS;
volatile bool watchdogActivated = true;
>>>>>>> memory-otimization
int MPU9150_I2C_ADDRESS = 0x68;


#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
SoftwareSerial serialLCD(6, 7);


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


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void delayZ(unsigned long timer){
  unsigned long previousMillis=millis();
  while((unsigned long)(millis() - previousMillis) < timer);
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

  uint32_i = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  uint8_i = level ? digitalPinToBitMask(DHT11_pin) : 0;
  while ((*portInputRegister(digitalPinToPort(DHT11_pin)) & digitalPinToBitMask(DHT11_pin)) == uint8_i) {
    if (uint32_i++ >= microsecondsToClockCycles(1000)) {
      return 0; // Exceeded timeout, fail.
    }
  }

  return uint32_i;
}

<<<<<<< HEAD
boolean readDHT11() {
  bool bool_i;
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
      bool_i = false;
      return bool_i;
    }
    if (expectPulse(HIGH) == 0) {
      bool_i = false;
      return bool_i;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (uint8_i = 0; uint8_i < 80; uint8_i += 2) {
      cycles[uint8_i]   = expectPulse(LOW);
      cycles[uint8_i + 1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (uint8_i = 0; uint8_i < 40; ++uint8_i) {
    uint32_i   = cycles[2 * uint8_i];
    uint32_j   = cycles[2 * uint8_i + 1];
    if ((uint32_i == 0) || (uint32_j == 0)) {
      bool_i = false;
      return bool_i;
    }
    dataDHT11[uint8_i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (uint32_j > uint32_i) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      dataDHT11[uint8_i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (dataDHT11[4] == ((dataDHT11[0] + dataDHT11[1] + dataDHT11[2] + dataDHT11[3]) & 0xFF)) {
    bool_i = true;

    itoa(dataDHT11[0], DHT11_hum_str, 10);
    itoa(dataDHT11[2], DHT11_temp_str, 10);

    return bool_i;
  }
  else {
    bool_i = false;
    return bool_i;
  }
}


=======
>>>>>>> memory-otimization
int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte_i = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte_j = Wire.read();

  return (int16_t)((byte_j<<8)+byte_i);
}


void MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return;
}



/***LOW LEVEL AT FONA COMMANDS***********************************************/

uint8_t ATreadFONA(uint8_t multiline = 0, int timeout = 10000) {

  uint16_i = 0;

  while (timeout--) {
    if (uint16_i >= 254) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (fonaSS.available()) {
      char_c =  fonaSS.read();
      if (char_c == '\r') continue;
      if (char_c == 0xA) {
        if (uint16_i == 0)   // the first 0x0A is ignored
          continue;

        if (!multiline--) {
          timeout = 0;         // the second 0x0A is the end of the line
          break;
        }


      }
      if (char_c == 0xA)
        dataBuffer[uint16_i] = ';';
      else
        dataBuffer[uint16_i] = char_c;
      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);
      uint16_i++;

    }

    //Serial.println(timeout);
    delay(1);
  }

  dataBuffer[uint16_i] = 0;  // null term
  Serial.print(F("READ: "));
  Serial.println(dataBuffer);
  //delay(1000);
  return uint16_i;
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


<<<<<<< HEAD
void getImeiFONA() {

  ATsendReadFONA(F("AT+GSN"), 2);
  char_pt1 = strtok(dataBuffer, ";");
  strcpy(IMEI_str, char_pt1);
}


uint8_t batteryCheckFONA() {

  ATsendReadFONA(F("AT+CBC"), 2);

  // typical string from FONA: "+CBC: 0,82,4057;OK"
  char_pt1 = strtok(dataBuffer, ":");
  char_pt1 = strtok(NULL, ",");
  batt_state = atoi(char_pt1);

  char_pt1 = strtok(NULL, ",");
  batt_percent = atoi(char_pt1);
  strcpy(batt_percent_str, char_pt1);

  char_pt1 = strtok(NULL, ";");
  batt_voltage = atoi(char_pt1);
  strcpy(batt_volt_str, char_pt1);

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
      int_i = 0;
      do {
        //while( dataBuffer[index] !='\n' ){
        dataBuffer[int_i] = SDfile.read();
      }
      while (dataBuffer[int_i++] != '\n');

      if ( dataBuffer[0] != '#') {

        char_pt1 = strtok(dataBuffer, "=");
        char_pt2 = strtok(NULL, "\n");

        if (! strcmp_P(char_pt2, (const char PROGMEM *)F("VOID")) == 0 ) {


          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("apn")) == 0 )
            strcpy(apn, char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("user")) == 0 )
            strcpy(user, char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("pwd")) == 0 )
            strcpy(pwd, char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("url")) == 0 )
            strcpy(url, char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("name")) == 0 )
            strcpy(NAME_str, char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
            LOGGING_FREQ_SECONDS = atoi(char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_WAIT")) == 0 )
            GPS_WAIT = atoi(char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
            GPS_FIX_MIN = atoi(char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_AVG")) == 0 )
            GPS_AVG = atoi(char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
            NUMBER_OF_DATA = atoi(char_pt2);

          if ( strcmp_P(char_pt1, (const char PROGMEM *)F("DEBUG")) == 0 )
            DEBUG = atoi(char_pt2);

          Serial.print("SD: ");
          Serial.print(char_pt1);
          Serial.print("=");
          Serial.println(char_pt2);
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

  uint8_i = GPS_WAIT;
  uint8_j;
  while (uint8_i--) {

    if ( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("+CGPSSTATUS: Location Not Fix;;OK"), 2) )
      uint8_j = 1;
    else if (dataBuffer[22] == '3')
      uint8_j = 3;
    else if (dataBuffer[22] == '2')
      uint8_j = 2;
    else if (dataBuffer[22] == 'U')
      uint8_j = 0;

    if (uint8_j >= GPS_FIX_MIN) {

      ATsendReadFONA(F("AT+CGPSINF=32"), 2);
      //strcpy(dataBuffer,"+CGPSINF: 32,061128.000,A,6209.9268,N,01710.7044,E,0.000,292.91,110915,");

      //-------------------------------
      // skip mode
      char_pt1 = strtok(dataBuffer, ",");
      if (! char_pt1) return false;

      // grab current UTC time hhmmss.sss ,-skip the last three digits.
      char_pt1 = strtok(NULL, ",");
      if (! char_pt1) return false;
      else strncpy(time_str, char_pt1, 6);

      // check fix
      char_pt1 = strtok(NULL, ",");
      if (! char_pt1) return false;

      if (!char_pt1[0] == 'xV')
        return false;


      // grab the latitude
      char_pt2 = strtok(NULL, ",");
      if (! char_pt2) return false;

      // grab latitude direction
      char_pt2 = strtok(NULL, ",");
      if (! char_pt2) return false;

      // grab longitude
      char_pt3 = strtok(NULL, ",");
      if (! char_pt3) return false;

      // grab longitude direction
      char_pt4 = strtok(NULL, ",");
      if (! char_pt4) return false;

      // skip speed
      char_pt1 = strtok(NULL, ",");
      if (! char_pt1) return false;

      // skip course
      char_pt1 = strtok(NULL, ",");
      if (! char_pt1) return false;

      // grab date ddmmyy
      char_pt1 = strtok(NULL, ",");
      if (! char_pt1) return false;
      else strcpy(date_str, char_pt1);

      double_i = atof(char_pt2);
      double_j = atof(char_pt3);

      // convert latitude from minutes to decimal
      double_k = floor(double_i / 100);
      double_l = double_i - (100 * double_k);
      double_l /= 60;
      double_k += double_l;

      // turn direction into + or -
      if (char_pt2[0] == 'S') double_k *= -1;

      dtostrf(double_k, 9, 5, latitude_str);
      lat = double_k;

      // convert longitude from minutes to decimal
      double_k = floor(double_j / 100);
      double_l = double_j - (100 * double_k);
      double_l /= 60;
      double_k += double_l;

      // turn direction into + or -
      if (char_pt4[0] == 'W') double_k *= -1;

      dtostrf(double_k, 9, 5, longitude_str);
      lon = double_k;
      //-------------------------
      //delay(2000);
      return uint8_j;
    }
    delay(2000);

  }
=======
>>>>>>> memory-otimization

//SETUP
//-------------------------------------------------------------------------------------------
void setup() {

  // MPU-9050 9-deg accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);



  // DHT11 temperature sensor digital pin
  pinMode(DHT11_pin, INPUT);
  digitalWrite(DHT11_pin, HIGH);

  serialLCD.begin(9600);
  delay(500);
  Serial.begin(115200);

  pinMode(FONA_PSTAT, INPUT);
  pinMode(FONA_POWER_KEY, OUTPUT);
  digitalWrite(FONA_POWER_KEY, HIGH);
<<<<<<< HEAD
  pinMode(FONA_POWER_KEY, OUTPUT);
  delay(500);
  */
  return true;
}


void clearInitData() {

  //char str1 = "IMEI=";

  eeprom_index = 0;

  eeprom_write_block(str1, &data[eeprom_index], 5);
  eeprom_index += 5;

  eeprom_write_block(IMEI_str, &data[eeprom_index], strlen(IMEI_str));
  eeprom_index += strlen(IMEI_str);

  //char str2 = "&name=";

  eeprom_write_block(str2, &data[eeprom_index], 6);
  eeprom_index += 6;

  eeprom_write_block(NAME_str, &data[eeprom_index], strlen(NAME_str));
  eeprom_index += strlen(NAME_str);
=======

  // This next section of code is timing critical, so interrupts are disabled.
  noInterrupts();
>>>>>>> memory-otimization

  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1 << WDRF);

<<<<<<< HEAD
  //char str3 = "&data=";
=======
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1 << WDCE) | (1 << WDE);
>>>>>>> memory-otimization

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

<<<<<<< HEAD
void saveData() {

  //char square[] = "#";
=======
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
>>>>>>> memory-otimization

    if (sleepIterations >= MAX_SLEEP_ITERATIONS_GPS) {

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======
      //AWAKE, -DO SOME WORK!
      //-----------------------------------------------------------------------      
      messageLCD(1000, F("ARDUINO"), F(">booting"));
>>>>>>> memory-otimization

      // Reset the number of sleep iterations.
      sleepIterations = 0;
      

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======
>>>>>>> memory-otimization

      // START UP FONA 808 MODULE
      //-----------------------------------------------------------------------
      fonaSS.begin(4800);
      fonaSS.flush();

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======
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
>>>>>>> memory-otimization

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
        ATsendReadFONA(F("ATE0"));
        delay(100);

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======
      } while (reset == true);
>>>>>>> memory-otimization

      messageLCD(0, F("FONA:"), F(">on"));

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
  
  eeprom_write_block(batt_percent_str, &data[eeprom_index], strlen(batt_percent_str));
  eeprom_index += strlen(batt_percent_str);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
  
  eeprom_write_block(DHT11_hum_str, &data[eeprom_index], 2);
  eeprom_index += 2;

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======


      // IS THIS FIRST RUN?, -THEN INIT/CLEAR EEPROM STORAGE and LOAD SDCARD
      //-----------------------------------------------------------------------
      if (samples == 1){
      
        // READ IMEI NUMBER OF SIM CARD
        //---------------------------------------------------------------------        
        ATsendReadFONA(F("AT+GSN"), 2);
        char* bufferPointer = strtok(dataBuffer, ";");
>>>>>>> memory-otimization

        // This is the start of the log, reset the index position to zero
        eeprom_index = 0;

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
=======
        // Write the first log note, the IMEI number of the unit
        eeprom_write_block("IMEI=", &data[eeprom_index], 5);
        eeprom_index += 5;
>>>>>>> memory-otimization

        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);
      
        messageLCD(0, F("FONA imei:"), bufferPointer);

<<<<<<< HEAD
  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;


  itoa(AcX, strA_8, 10);

  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;

  itoa(AcY, strA_8, 10);

  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;

  itoa(AcZ, strA_8, 10);
  
  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;


  itoa(MaX, strA_8, 10);

  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
  
  itoa(MaY, strA_8, 10);
  
  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;

  itoa(MaZ, strA_8, 10);
  
  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);

  eeprom_write_block("#", &data[eeprom_index], 1);
  eeprom_index += 1;
}
=======


        // LOAD USER CONFIGUARTION FROM SDCARD
        //---------------------------------------------------------------------     
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

                //Write the second log note, the name of the horse
                if ( strcmp_P(parameter, (const char PROGMEM *)F("name")) == 0 ){
                  eeprom_write_block("&name=", &data[eeprom_index], 6);
                  eeprom_index += 6;

                  eeprom_write_block(value, &data[eeprom_index], strlen(value));
                  eeprom_index += strlen(value);
                }

                if ( strcmp_P(parameter, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
                  LOGGING_FREQ_SECONDS = atoi(value);

                if ( strcmp_P(parameter, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
                  GPS_FIX_MIN = atoi(value);
>>>>>>> memory-otimization

                if ( strcmp_P(parameter, (const char PROGMEM *)F("GPS_AVG")) == 0 )
                  GPS_AVG = atoi(value);

                if ( strcmp_P(parameter, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
                  NUMBER_OF_DATA = atoi(value);



                Serial.print("SD: ");
                Serial.print(parameter);
                Serial.print("=");
                Serial.println(value);
              }
            }
          }
          SDfile.close();
        }

        // Write the third log note, the start of the measurement data!
        // This finish the init of the data log.
        eeprom_write_block("&data=", &data[eeprom_index], 6);
        eeprom_index += 6;
        }


        
        // CHECK BATTERY LEVEL
        //-----------------------------------------------------------------------     
        delay(100);
        
        ATsendReadFONA(F("AT+CBC"), 2);

        // typical string from FONA: "+CBC: 0,82,4057;OK"
        char *bufferPointer = strtok(dataBuffer, ":");
        // skip charge state of battery
        bufferPointer = strtok(NULL, ",");
        

<<<<<<< HEAD
  // add final parameter, -the bit-checksum
  //char str1[] = "&sum=";
  eeprom_write_block(str4, &data[eeprom_index], 5);
  eeprom_index += 5;
=======
        // read charge percent of battery
        bufferPointer = strtok(NULL, ",");
>>>>>>> memory-otimization

        
        messageLCD(1000, "Batt%", bufferPointer );
        
        // save charge percent to log
        char square[] ="#";
        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);

<<<<<<< HEAD
  uint16_j = 0;
  //char pop[1];
  for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
    eeprom_read_block(pop, &data[uint16_i], 1);
    uint16_j += __builtin_popcount(pop[0]);
  }
  //char sum_str[6];

  itoa(uint16_j, strA_8, 10);
  strcat(strA_8, "&");

  eeprom_write_block(strA_8, &data[eeprom_index], strlen(strA_8));
  eeprom_index += strlen(strA_8);


  for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
    eeprom_read_block(pop, &data[uint16_i], 1);
    Serial.write(*pop);
    //if((i-5)%40 == 0)
  }
  Serial.write('\n');

  // setup length of data to send
  strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));
  //char dataLengthStr[4];

  itoa(eeprom_index + 1, strA_8, 10);
  strcat(dataBuffer, strA_8);
=======
        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

        // read battery millivoltage
        bufferPointer = strtok(NULL, ";");
  
        // save battery voltage to log
        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
        


        // READ DATA FROM TEMP/HUMID SENSOR DHT11
        //-----------------------------------------------------------------------

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
          expectPulse(LOW);
          expectPulse(HIGH);

          // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
          // microsecond low pulse followed by a variable length high pulse.  If the
          // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
          // then it's a 1.  We measure the cycle count of the initial 50us low pulse
          // and use that to compare to the cycle count of the high pulse to determine
          // if the bit is a 0 (high state cycle count < low state cycle count), or a
          // 1 (high state cycle count > low state cycle count). Note that for speed all
          // the pulses are read into a array and then examined in a later step.
          for (uint8_t i = 0; i < 80; i += 2) {
            cycles[i]   = expectPulse(LOW);
            cycles[i + 1] = expectPulse(HIGH);
          }
        } // Timing critical code is now complete.

        // Inspect pulses and determine which ones are 0 (high state cycle count < low
        // state cycle count), or 1 (high state cycle count > low state cycle count).
        for (uint8_t i = 0; i < 40; ++i) {
          uint32_t lowCycles  = cycles[2 * i];
          uint32_t highCycles = cycles[2 * i + 1];

       
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
        
        // Write DHT11 data to log
        itoa(dataDHT11[0], str10_A, 10);
        
        eeprom_write_block(str10_A, &data[eeprom_index], 2);
        eeprom_index += 2;


        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;         
          
        itoa(dataDHT11[2], str10_A, 10);
       
        eeprom_write_block(str10_A, &data[eeprom_index], 2);
        eeprom_index += 2;

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

        // READ DATA FROM ACCELEROMETER MPU9150
        //-----------------------------------------------------------------------    
        // INIT MPU-9150  

        MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B00000010);     //Wake up MPU
        MPU9150_writeSensor(MPU9150_SMPLRT_DIV, B00000001);     //Set sample rate divider
        MPU9150_writeSensor(MPU9150_CONFIG, 6);                 //Set lowpass filter to 5Hz
        MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, B00000000);   //Set accelerometer range to +/-2g
        MPU9150_writeSensor(MPU9150_USER_CTRL, B00000000);      //Disable MPU as master for I2C slave
        MPU9150_writeSensor(MPU9150_INT_PIN_CFG, B00110010);    //Set bypass mode on I2C slave
        MPU9150_writeSensor(MPU9150_PWR_MGMT_2, B00000111);     //Put xyz gyros to standby
        
        // READ MPU-9150 TEMP DATA        
        MPU9150_I2C_ADDRESS = 0x68;
        delay(10);
        int16_i = MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H)/340+36.53;
        dtostrf(int16_i, 5, 1, str10_A);
  
        // WRITE MPU TEMP TO LOG
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
>>>>>>> memory-otimization

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

<<<<<<< HEAD
  // downloading data to send to FONA
  for ( uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
    eeprom_read_block(pop, &data[uint16_i], 1);
    fonaSS.write(pop[0]);
    Serial.write(pop[0]);
  }
  fonaSS.write('\n');
  Serial.write('\n');
=======
        // READ MPU-9150 ACCELERATION DATA
>>>>>>> memory-otimization

        // read acceleration in X-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
        itoa(int16_i, str10_A, 10);

        // write X acceleration to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

<<<<<<< HEAD
  // read reply from server, HTTP code
  ATreadFONA(0, 11000);
  char_pt1 = strtok(dataBuffer, ",");
  char_pt1 = strtok(NULL, ",");
  strcpy(error_code, char_pt1);

  if ( atoi(char_pt1) == 200 || atoi(char_pt1) == 302 )
    return true;
  else
    return false;
}
=======
        // read acceleration in Y-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
        itoa(int16_i, str10_A, 10);
        
        // write Y acceleration to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // read acceleration in Z-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
        itoa(int16_i, str10_A, 10);
        
        // write Z acceleration to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
>>>>>>> memory-otimization

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
        

        // READ MPU-9150 MAGNETO/COMPASS DATA
        MPU9150_I2C_ADDRESS = 0x0c;
  
        MPU9150_writeSensor(0x0A, 0x02);
        delay(10);

        // read compass data in X direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
        itoa(int16_i, str10_A, 10);

        // write compass X to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
  
        // read compass data in Y direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
        itoa(int16_i, str10_A, 10);

        // write compass Y to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // read compass data in Z direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
        itoa(int16_i, str10_A, 10);        
        
        // write compass Z to log
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;


        // SLEEP MPU-9150
        delay(10);
        MPU9150_I2C_ADDRESS = 0x0C; 
        MPU9150_writeSensor(0x0A, B00000000);
        MPU9150_I2C_ADDRESS = 0x68; 
        MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B01000000);



        // TURN ON THE GPS UNIT IN FONA MODULE
        //-----------------------------------------------------------------------
        messageLCD(0, F("FONA-gps"), F(">on"));
        
        // first check if GPS is  on or off, if off, -turn it on
        if( ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 0;;OK"), 2) )
          ATsendReadFONA(F("AT+CGPSPWR=1"));
  


        //READ GPS LOCATION DATA of the FONA 808 UNIT
        //-----------------------------------------------------------------------  
        uint8_t fix_status; 
        for (uint8_t i = 0; i < POS_SIZE; i) {
          
          uint8_t timeout = GPS_WAIT;    
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

              // skip mode
              bufferPointer = strtok(dataBuffer, ",");

              // grab current UTC time hhmmss.sss ,-skip the last three digits.
              bufferPointer = strtok(NULL, ",");
              strncpy(str10_B, bufferPointer, 6);

              // skip valid fix
              bufferPointer = strtok(NULL, ",");

              // grab the latitude
              char *latp = strtok(NULL, ",");

              // grab latitude direction
              char *latdir = strtok(NULL, ",");

              // grab longitude
              char *longp = strtok(NULL, ",");

              // grab longitude direction
              char *longdir = strtok(NULL, ",");

              // skip speed
              bufferPointer = strtok(NULL, ",");

              // skip course
              bufferPointer = strtok(NULL, ",");

              // grab date ddmmyy
              bufferPointer = strtok(NULL, ",");
              strcpy(str10_C, bufferPointer);

              float latitude = atof(latp);
              float longitude = atof(longp);

              // convert latitude from minutes to decimal
              float degrees = floor(latitude / 100);
              float minutes = latitude - (100 * degrees);
              minutes /= 60;
              degrees += minutes;

              // turn direction into + or -
              if (latdir[0] == 'S') degrees *= -1;

              lat = degrees;

              // convert longitude from minutes to decimal
              degrees = floor(longitude / 100);
              minutes = longitude - (100 * degrees);
              minutes /= 60;
              degrees += minutes;

              // turn direction into + or -
              if (longdir[0] == 'W') degrees *= -1;

              lon = degrees;
            }
            if( fix_status>=GPS_FIX_MIN )
              break;
            else
              delay(1000);
          }

          if( fix_status>=GPS_FIX_MIN ){
            
            messageLCD(0, F("FONA-gps"), ">get #" + String(i+1) );
            latArray[i]=lat;
            lonArray[i]=lon;
            delay(100);        
            i++;    
          }


<<<<<<< HEAD
      // CHECK BATTERY LEVEL
      delay(100);
      batteryCheckFONA();
      messageLCD(1000, "Battery %", String(batt_percent) );
      
      for (uint8_k = 0; uint8_k < POS_SIZE; uint8_k) {
        if( readGpsFONA808() ){
          messageLCD(0, F("FONA-gps"), ">get #" + String(uint8_k+1) );
          latArray[uint8_k]=lat;
          lonArray[uint8_k]=lon;
          delay(4000);
          //t0=t0+9;
          uint8_k++;
        }
        
      }
      for (uint8_i = 0; uint8_i < POS_SIZE; uint8_i++)  
        for (uint8_j = 1; uint8_j < POS_SIZE-uint8_i; uint8_j++) {
          if( latArray[uint8_j]<=latArray[uint8_j-1] ){
            double latTemp = latArray[uint8_j];
            latArray[uint8_j] = latArray[uint8_j-1];
            latArray[uint8_j-1] = latTemp;
          }
          if( lonArray[uint8_j]<=lonArray[uint8_j-1] ){
            double lonTemp = lonArray[uint8_j];
            lonArray[uint8_j] = lonArray[uint8_j-1];
            lonArray[uint8_j-1] = lonTemp;
          }
          
=======
        
        //CALCULATE THE MEDIAN VALUE OF THE LOCATION DATA
        //-----------------------------------------------------------------------          
>>>>>>> memory-otimization
        }
        for (uint8_t i = 0; i < POS_SIZE; i++)  
          for (uint8_t j = 1; j < POS_SIZE-i; j++) {
            if( latArray[j]<=latArray[j-1] ){
              float latTemp = latArray[j];
              latArray[j] = latArray[j-1];
              latArray[j-1] = latTemp;
            }
            if( lonArray[j]<=lonArray[j-1] ){
              float lonTemp = lonArray[j];
              lonArray[j] = lonArray[j-1];
              lonArray[j-1] = lonTemp;
            }
          
          }
        
        
        /*
        // TAKE GPS_AVG of GPS's READING FOLLOWED by AVERAGING
        lonAVG = 0;
        latAVG = 0;
        
        float lat1 = 0;
        float lat2 = 0;
        float lat3 = 0;
        float lat4 = 0;
        float lat5 = 0;
        float lat6 = 0;
        float lon1 = 0;
        float lon2 = 0;
        float lon3 = 0;
        float lon4 = 0;
        float lon5 = 0;
        float lon6 = 0;
        for (int i = 1; i <= GPS_AVG; i) {

          messageLCD(0, F("FONA-gps"), ">get #" + String(i) );

          if( readGpsFONA808() ){
          
            if( lat > lat3 )
             if( lat > lat4 )
              if( lat > lat5 ){
                lat1 = lat2;
                lat2 = lat3;
                lat3 = lat4;
                lat4 = lat5;
                lat5 = lat;  
              }
              else{
                lat1 = lat2;
                lat2 = lat3;
                lat3 = lat4;
                lat4 = lat;
              }
            else{
              lat1 = lat2;
              lat2 = lat3;
              lat3 = lat;
            }
              
          else
            if( lat < lat2 )
              if( lat < lat1 ){
                lat5 = lat4;
                lat4 = lat3;
                lat3 = lat2;
                lat2 = lat1;
                lat1 = lat;     
              }
              else{
                lat5 = lat4;
                lat4 = lat3;
                lat3 = lat2;
                lat2 = lat;                
              }
            else{
              lat3 = lat;
            }

          if( lon > lon3 )
            if( lon > lon4 )
              if( lon > lon5 ){
                lon1 = lon2;
                lon2 = lon3;
                lon3 = lon4;
                lon4 = lon5;
                lon5 = lon;  
              }
              else{
                lon1 = lon2;
                lon2 = lon3;
                lon3 = lon4;
                lon4 = lon;
              }
            else{
              lon1 = lon2;
              lon2 = lon3;
              lon3 = lon;
            }
              
          else
            if( lon < lon2 )
              if( lon < lon1 ){
                lon5 = lon4;
                lon4 = lon3;
                lon3 = lon2;
                lon2 = lon1;
                lon1 = lon;     
              }
              else{
                lon5 = lon4;
                lon4 = lon3;
                lon3 = lon2;
                lon2 = lon;                
              }
            else{
              lon3 = lon;
            }              
          

          //latAVG += lat;
          //lonAVG += lon;
          delay(2000);
          if( i==1 ){
           lat1 = lat;
           lat2 = lat;
           lat3 = lat;
           lat4 = lat;
           lat5 = lat; 
           lon1 = lon;
           lon2 = lon;
           lon3 = lon;
           lon4 = lon;
           lon5 = lon;        
          }
          
          }
          i++;
        }
      
        //Serial.print(lat1,5);Serial.print(" ");Serial.print(lat2,5);Serial.print(" ");Serial.print(lat3,5);
        //Serial.print(" ");Serial.print(lat4,5);Serial.print(" ");Serial.println(lat5,5);
        latAVG = lat3;//(lat2 + lat3 + lat4)/3;
        lonAVG = lon3;//(lon2 + lon3 + lon4)/3;
        //latAVG/=GPS_AVG;
        //lonAVG/=GPS_AVG;
        */
        //01234 5 67890
        //0123456 7 8901234
        //0123456789 0 1234567890
        
        //latAVG=latArray[int(POS_SIZE/2)];
        //lonAVG=lonArray[int(POS_SIZE/2)];

        // WRTIE LATITUDE GPS DATA TO LOG
        dtostrf(latArray[int(POS_SIZE/2)], 9, 5, str10_A);

        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

        // WRTIE LONGITUDE GPS DATA TO LOG
        dtostrf(lonArray[int(POS_SIZE/2)], 9, 5, str10_A);
        
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;

        // WRTIE GPS DATE TO LOG
        eeprom_write_block(str10_B, &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // WRTIE GPS TIME TO LOG
        eeprom_write_block(str10_C, &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(square, &data[eeprom_index], 1);
        eeprom_index += 1;



        // TIME TO SEND DATA TO SERVER?
        //----------------------------------------------------------------------- 
        if ( samples >= NUMBER_OF_DATA ) {
          


        // TURN ON GPRS
        //-----------------------------------------------------------------------         
        ATsendReadFONA(F("AT+CIPSHUT"));

        ATsendReadFONA(F("AT+CGATT?"), 2);

        ATsendReadFONA(F("AT+CGATT=0"));

        ATsendReadFONA(F("AT+CGATT=1"));

        ATsendReadFONA(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));
         

        strcpy(dataBuffer, "AT+SAPBR=3,1,\"APN\",\"");

        strcat(dataBuffer, apn);
        strcat(dataBuffer, "\"");
        ATsendReadFONA(dataBuffer);

        if (user == "") {
          strcpy(dataBuffer, "AT+SAPBR=3,1,\"USER\",\"");
          strcat(dataBuffer, user);
          strcat(dataBuffer, "\"");
          ATsendReadFONA(dataBuffer);
        }

        if (pwd == "") {
          strcpy(dataBuffer, "AT+SAPBR=3,1,\"PWD\",\"");
          strcat(dataBuffer, pwd);
          strcat(dataBuffer, "\"");
          ATsendReadFONA(dataBuffer);
        }

        ATsendReadFONA(F("AT+SAPBR=1,1"));

        messageLCD(0, F("FONA-gprs"), F(">on"));



        // SENDING DATA BY HTTP POST
<<<<<<< HEAD
        //strA_8 = "000";
        if ( sendDataServer(strA_8) )
          messageLCD(2000, F("HTTP"), ">OK #" + String(strA_8));
        else
          messageLCD(2000, F("HTTP"), ">ERR #" + String(strA_8));
=======
        //-----------------------------------------------------------------------         
        char error_code[4] = "000";
>>>>>>> memory-otimization

        // close all prevoius HTTP sessions
        ATsendReadFONA(F("AT+HTTPTERM"));

        // start a new HTTP session
        ATsendReadFONA(F("AT+HTTPINIT")); 
   
        // setup the HTML HEADER
        // CID = Bearer profile identifier =
        ATsendReadFONA(F("AT+HTTPPARA=\"CID\",\"1\""));
   
        // setup the HTML USER AGENT
        ATsendReadFONA(F("AT+HTTPPARA=\"UA\",\"CLOUDMARE1.0\""));
    
        // setup the HTML CONTENT
        ATsendReadFONA(F("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""));
    
        // setup URL to send data to
        strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPPARA=\"URL\",\""));
        strcat(dataBuffer, url);
        strcat(dataBuffer, "\"");
        ATsendReadFONA(dataBuffer);
    
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
        }
        Serial.write('\n');

        // setup length of data to send
        strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));
        char dataLengthStr[4];

        itoa(eeprom_index + 1, dataLengthStr, 10);
        strcat(dataBuffer, dataLengthStr);

        strcat(dataBuffer, ",");
          strcat_P(dataBuffer, (const char PROGMEM *)F("10000"));
          ATsendReadFONA(dataBuffer, 0);

          // downloading data to send to FONA
          for ( uint16_t i = 0; i < eeprom_index; i++) {
            eeprom_read_block(pop, &data[i], 1);
            fonaSS.write(pop[0]);
            //Serial.write(pop[0]);
          }
          fonaSS.write('\n');
          //Serial.write('\n');

          // Check if download was OK? (-eat up OK)
          ATreadFONA();

          // sending data by HTTP POST
          ATsendReadFONA(F("AT+HTTPACTION=1"));

          // read reply from server, HTTP code
          ATreadFONA(0, 11000);
          bufferPointer = strtok(dataBuffer, ",");
          bufferPointer = strtok(NULL, ","); 
          if ( bufferPointer == "302" || bufferPointer == "200" )
            messageLCD(2000, F("HTTP"), ">OK #" + String(bufferPointer) );
          else
            messageLCD(2000, F("HTTP"), ">ERR #" + String(bufferPointer) );

          samples = 0;



          // TURN OFF GPRS
          //-----------------------------------------------------------------------
          ATsendReadFONA(F("AT+CIPSHUT"));

          ATsendReadFONA(F("AT+CGATT?"), 2);
        }
        samples++;



        // POWER DOWN FONA
        //-----------------------------------------------------------------------  

        // first check if GPS is already on or off, if so, -shut it down!
        if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;;OK"), 2) ) {
          ATsendReadFONA(F("AT+CGPSPWR=0"));
        }
        ATsendReadFONA(F("AT+CPOWD=1"));


        // GO TO SLEEP
        messageLCD(-1000, F("ARDUINO"), F(">sleep"));
      }
    }

    sleep();

}

