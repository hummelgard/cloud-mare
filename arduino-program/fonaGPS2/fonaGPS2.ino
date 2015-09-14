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
//#include <avr/eeprom.h>

//#include "crc16.h"

#include <avr/pgmspace.h>
// next line per http://postwarrior.com/arduino-ethershield-error-prog_char-does-not-name-a-type/
#define prog_char  char PROGMEM

// MPU-9050
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


// Seconds to wait before a new sensor reading is logged.
//#define LOGGING_FREQ_SECONDS   120

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

uint8_t GPS_WAIT       =0;
uint8_t GPS_AVG        =0;
uint8_t GPS_FIX_MIN    =0;
uint8_t NUMBER_OF_DATA =0;
uint8_t LOGGING_FREQ_SECONDS = 0;
uint8_t DEBUG = 0;

uint8_t dataDHT11[6];
uint8_t temperature = 0;
uint8_t humidity = 0;

// DEBUG levels, by hardware port 3 to set to high, level 3 can be set.
// Level 0=off, 1=some, 2=more, 3=most, 4=insane!



uint8_t samples=1;
char dataBuffer[80];

char data[26+45*3+12] = {0};

// USED BY: sendDataServer loadConfigSDcard
char url[] = "http://cloud-mare.hummelgard.com:88/addData";

// USED BY: sendDataServer
char    IMEI_str[17] = "123456789012345";
                      //865067020395128
uint8_t  batt_state;
uint8_t  batt_percent;
uint16_t  batt_voltage;

// USED BY: loadConfigSDcard sendDataServer enableGprsFONA
char apn[30] = {0};
char user[15] = {0};
char pwd[15] = {0};


// USED BY: readGpsFONA808
char latitude_str[12] = {0};
char longitude_str[12] = {0};

double lat;
double lon;
double latAVG;
double lonAVG;

char fix_qualityAVG_str[5] = {0};
char date_str[7] = {0};
char time_str[7] = {0};

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
  serialLCD.write(254);
  serialLCD.write(128);
  serialLCD.print("                ");
  serialLCD.write(254);
  serialLCD.write(192);
  serialLCD.print("                ");
  delay(10);
  serialLCD.write(124); //max brightness
  serialLCD.write(157);
  delay(10);
  serialLCD.write(254);
  serialLCD.write(128);
  serialLCD.print(line1);
  serialLCD.write(254);
  serialLCD.write(192);
  serialLCD.print(line2);
  if(time > 0)
    delay(time);
  else if(time < 0) {
    delay(-time);
    serialLCD.write(254);
    serialLCD.write(128);
    serialLCD.print("                ");
    serialLCD.write(254);

    serialLCD.write(192);
    serialLCD.print("                ");
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

boolean readDHT11(){
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
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      _lastresult = false;
      return _lastresult;
    }
    dataDHT11[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      dataDHT11[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (dataDHT11[4] == ((dataDHT11[0] + dataDHT11[1] + dataDHT11[2] + dataDHT11[3]) & 0xFF)) {
    _lastresult = true;

  humidity=dataDHT11[0];
  temperature=dataDHT11[2];  
  //dtostrf(data[2], 5, 1, temp_str);
  //dtostrf(data[0], 2, 0, hum_str);

    return _lastresult;
  }
  else {
    _lastresult = false;
    return _lastresult;
  }
}




/***LOW LEVEL AT FONA COMMANDS***********************************************/

uint8_t ATreadFONA(uint8_t multiline = 0, int timeout = 10000) {

  uint8_t replyidx = 0;
  while (timeout--) {

    while (fonaSS.available()) {
      char c =  fonaSS.read();
      if(c == '\r') continue;
      if(c == 0xA) {
        if(replyidx == 0)   // the first 0x0A is ignored
          continue;

        if( multiline == 0 ) {
          timeout = 0;
          continue;
        }
        if( multiline > 0 ) {
          dataBuffer[replyidx++] = ';';
          multiline--;
          continue;
        }
      }
      dataBuffer[replyidx] = c;
      #ifdef OLD_DEBUG
      if(DEBUG == 4) {
        Serial.print(F("\t\t\t")); Serial.print(c, HEX); Serial.print(F("#")); Serial.println(c);
      }
      else
      #endif
        delay(20);

      replyidx++;
    }
    delay(1);
  }
  dataBuffer[replyidx] = 0;  // null term
  //#ifdef OLD_DEBUG
  if(DEBUG >= 3) {
    //messageLCD(2000,"",dataBuffer);
    Serial.print(F("\t\tREAD: "));
    Serial.println(dataBuffer);
  }
  //#endif
  return replyidx;

}

uint8_t ATsendReadFONA(char* ATstring, uint8_t multiline = 0, int timeout = 10000) {

  //#ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    if(DEBUG >= 3)
      ;//messageLCD(2000, String(ATstring));
    Serial.print(F("\t\tSEND: "));
    Serial.println(String(ATstring));
  }
  //#endif
  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}

uint8_t ATsendReadFONA(const __FlashStringHelper *ATstring, uint8_t multiline = 0, int timeout = 10000) {

  //#ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    if(DEBUG >= 3)
      ;//messageLCD(2000, String(ATstring));
    Serial.print(F("\t\tSEND: "));
    Serial.println(String(ATstring));
  }
  //#endif
  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}

boolean ATsendReadVerifyFONA(char* ATstring, char* ATverify, uint8_t multiline = 0, int timeout = 10000) {

  if(ATsendReadFONA(ATstring, multiline, timeout)) {
    if( strcmp(dataBuffer, ATverify) == 0 )
      return true;
    else
      return false;
  }
}

boolean ATsendReadVerifyFONA(char* ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0, int timeout = 10000) {

  if(ATsendReadFONA(ATstring, multiline, timeout)) {
    if( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}

boolean ATsendReadVerifyFONA(const __FlashStringHelper *ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0, int timeout = 10000) {
  if(ATsendReadFONA(ATstring, multiline, timeout)) {
    if( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}


void getImeiFONA(){
  
  ATsendReadFONA(F("AT+GSN"),1);
  char* tok = strtok(dataBuffer,";");
  strcpy(IMEI_str, tok);
}


uint8_t batteryCheckFONA() {

  ATsendReadFONA(F("AT+CBC"), 1);

  // typical string from FONA: "+CBC: 0,82,4057;OK"
  char* tok = strtok(dataBuffer, ":");
  tok = strtok(NULL, ",");
  batt_state = atoi(tok);

  tok = strtok(NULL, ",");
  batt_percent = atoi(tok);

  tok = strtok(NULL, "\n");
  batt_voltage = atoi(tok);

  #ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    Serial.print(F("\t\tFONA BATTERY: "));
    Serial.print(batt_state);
    Serial.print(F(", "));
    Serial.print(batt_percent);
    Serial.print(F("%, "));
    Serial.print(batt_voltage);
    Serial.println(F("mV"));
  }
  #endif
  return batt_percent;
}

/***GPRS COMMANDS************************************************************/
boolean loadConfigSDcard() {
  File SDfile;
  SdFat SD;
  SD.begin(SDCARD_CS);
  SDfile = SD.open("config.txt");
  if(SDfile) {

    while (SDfile.available()) {

      // read one line at a time
      int index = 0;
      do {
        //while( dataBuffer[index] !='\n' ){
        dataBuffer[index] = SDfile.read();
      }
      while (dataBuffer[index++] != '\n');

      if( dataBuffer[index] !='#'){
        
        char* parameter = strtok(dataBuffer, "=");
        char* value = strtok(NULL, "\n");
      
        if(! strcmp_P(value, (const char PROGMEM *)F("VOID"))==0 ){
        //Serial.print("parameter="); Serial.println(parameter);
        //Serial.print("value="); Serial.println(value);
  
        if( strcmp_P(parameter, (const char PROGMEM *)F("apn")) == 0 )
          strcpy(apn, value);
      
        if( strcmp_P(parameter, (const char PROGMEM *)F("user")) == 0 )
          strcpy(user, value);
        
        if( strcmp_P(parameter, (const char PROGMEM *)F("pwd")) == 0 )
          strcpy(pwd, value);
        
        if( strcmp_P(parameter, (const char PROGMEM *)F("url")) == 0 )
          strcpy(url, value);
        
        if( strcmp_P(parameter, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
          LOGGING_FREQ_SECONDS = atoi(value);       

        if( strcmp_P(parameter, (const char PROGMEM *)F("GPS_WAIT")) == 0 )
          GPS_WAIT = atoi(value);

        if( strcmp_P(parameter, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
          GPS_FIX_MIN = atoi(value);        

        if( strcmp_P(parameter, (const char PROGMEM *)F("GPS_AVG")) == 0 )
          GPS_AVG = atoi(value);        

        if( strcmp_P(parameter, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
          NUMBER_OF_DATA = atoi(value);  
                
        if( strcmp_P(parameter, (const char PROGMEM *)F("DEBUG")) == 0 )
          DEBUG = atoi(value);  

        }
      }
    }
    SDfile.close();

    #ifdef OLD_DEBUG
    if(DEBUG >= 2) {
      messageLCD(2000, F("SDcard"), F("load config"));
      Serial.print(F("\t\tSDcard: apn="));
      Serial.println(apn);
      Serial.print(F("\t\tSDcard: user="));
      Serial.println(user);
      Serial.print(F("\t\tSDcard: pwd="));
      Serial.println(pwd);
      Serial.print(F("\t\tSDcard: url="));
      Serial.println(url);
      Serial.print(F("\t\tSDcard: LOGGING_FREQ_SECONDS="));
      Serial.println(LOGGING_FREQ_SECONDS);
      Serial.print(F("\t\tSDcard: GPS_WAIT="));
      Serial.println(GPS_WAIT);
      Serial.print(F("\t\tSDcard: GPS_FIX_MIN="));
      Serial.println(GPS_FIX_MIN);
      Serial.print(F("\t\tSDcard: GPS_AVG="));
      Serial.println(GPS_AVG);
      Serial.print(F("\t\tSDcard: NUMBER_OF_DATA="));
      Serial.println(NUMBER_OF_DATA);
      Serial.print(F("\t\tSDcard: DEBUG="));
      Serial.println(DEBUG);
    
    }
    #endif
  }
  return true;
}


boolean disableGprsFONA(){

  if(! ATsendReadVerifyFONA(F("AT+CIPSHUT"), F("SHUT OK")) )
    return false;

  if(! ATsendReadVerifyFONA(F("AT+CGATT?"), F("+CGATT: 0;OK"), 1) )
    return false;
}


boolean enableGprsFONA() {


  if(! ATsendReadVerifyFONA(F("AT+CIPSHUT"), F("SHUT OK")) )
    return false;

  if( ATsendReadVerifyFONA(F("AT+CGATT?"), F("+CGATT: 1;OK"), 1) ) {
    #ifdef OLD_DEBUG
    if(DEBUG >= 1) {
      messageLCD(2000, "FONA gprs on", ">restart");
      Serial.println("\tFONA gprs is already on, -restarting");
      
      // shut it down
      #endif
      if(! ATsendReadVerifyFONA(F("AT+CGATT=0"), F("OK")) )
        return false; 
      //return true;
    //}
  //}
  
  #ifdef OLD_DEBUG
  else {
    if(DEBUG >= 1) {
      messageLCD(2000, "FONA gprs off", ">starting up");
      Serial.println("\tFONA gprs is off, >starting up");
    }    
  #endif  
  }
  
  if(! ATsendReadVerifyFONA(F("AT+CGATT=1"), F("OK")) )
    return false;

  if(! ATsendReadVerifyFONA(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), F("OK")) )
    return false;

  strcpy(dataBuffer, "AT+SAPBR=3,1,\"APN\",\"");

  strcat(dataBuffer, apn);
  strcat(dataBuffer, "\"");
  if(! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
    return false;

  if(user == "") {
    strcpy(dataBuffer, "AT+SAPBR=3,1,\"USER\",\"");
    strcat(dataBuffer, user);
    strcat(dataBuffer, "\"");
    if(! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
      return false;
  }

  if(pwd == "") {
    strcpy(dataBuffer, "AT+SAPBR=3,1,\"PWD\",\"");
    strcat(dataBuffer, pwd);
    strcat(dataBuffer, "\"");
    if(! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
      return false;
  }

  if(! ATsendReadVerifyFONA(F("AT+SAPBR=1,1"), F("OK")) )
    return false;


  return true;
}

boolean initFONA() {

  fonaSS.begin(4800);

  // Check if FONA os ON, if not turn it on!
  if(digitalRead(FONA_PSTAT) == false ) {
    #ifdef OLD_DEBUG
    if(DEBUG >= 2) {
      messageLCD(1000, F("FONA: off"), F(">power on"));
      Serial.println(F("\tFONA is off, >power on"));
    }
    #endif
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
    if( reset == true ) {
      #ifdef OLD_DEBUG
      if(DEBUG >= 2) {
        messageLCD(1000, F("FONA: error"), F(">reseting"));
        Serial.println(F("\tFONA error, >reseting"));
      }
      #endif
      pinMode(FONA_RST, OUTPUT);
      digitalWrite(FONA_RST, HIGH);
      delay(100);
      digitalWrite(FONA_RST, LOW);
      delay(100);
      digitalWrite(FONA_RST, HIGH);
      delay(7000);

      reset = false;
    }

    if(! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;

    if(! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;

    if(! ATsendReadVerifyFONA(F("AT"), F("OK")) )
      reset = true;

    // turn off Echo!
    if(! ATsendReadVerifyFONA(F("ATE0"), F("OK")) )
      reset = true;

    // turn on hangupitude
    if(! ATsendReadVerifyFONA(F("AT+CVHU=0"), F("OK")) )
      reset = true;

  } while (reset == true);
  #ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    messageLCD(1000, F("FONA: init"), F(">OK"));
    Serial.println(F("\tFONA init, >OK"));
  }
  #endif
  return true;
}


boolean enableGpsFONA808(void) {

  // first check if GPS is already on or off
  if(ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;OK"), 1) ) {
    #ifdef OLD_DEBUG
    if(DEBUG >= 2) {
      messageLCD(1000, F("GPS power: on"), F(">OK"));
      Serial.println(F("\tFONA GPS is already power on"));
    }
    #endif
    return false;
  }
  else {
    #ifdef OLD_DEBUG
    if(DEBUG >= 2) {
      messageLCD(1000, F("GPS power: off"), F(">power on"));
      Serial.println(F("\tFONA GPS power is off, turning it on"));
    }
    #endif
    if(! ATsendReadVerifyFONA(F("AT+CGPSPWR=1"), F("OK")) )
      return false;
  }

  return true;


}

uint8_t readGpsFONA808(){

    //READ: +CGPSINF: 32,061128.000,A,6209.9268,N,01710.7044,E,0.000,292.91,110915,;

  uint8_t timeout = GPS_WAIT;
  uint8_t fix_status;
  while (timeout--) {

    if( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("+CGPSSTATUS: Location Not Fix;OK"), 1) )
      fix_status = 1;
    else if(dataBuffer[22] == '3')
      fix_status = 3;
    else if(dataBuffer[22] == '2')
      fix_status = 2;
    else if(dataBuffer[22] == 'U')
      fix_status = 0;

    if(fix_status >= GPS_FIX_MIN) {
      
      ATsendReadFONA(F("AT+CGPSINF=32"), 1);
      //strcpy(dataBuffer,"+CGPSINF: 32,061128.000,A,6209.9268,N,01710.7044,E,0.000,292.91,110915,");
      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {
        Serial.print(F("\tFONA GPSdata: "));
        Serial.print(fix_status);
        Serial.print(F(" RMC: "));
        Serial.println(dataBuffer);
      }
      #endif
      //-------------------------------
      // skip mode
      char *tok = strtok(dataBuffer, ",");
      if(! tok) return false;

      // grab current UTC time hhmmss.sss ,-skip the last three digits.
      tok = strtok(NULL, ",");
      if(! tok) return false;
      else strncpy(time_str,tok,6);

      // skip fix
      tok = strtok(NULL, ",");
      if(! tok) return false;

      // grab the latitude
      char *latp = strtok(NULL, ",");
      if(! latp) return false;

      // grab latitude direction
      char *latdir = strtok(NULL, ",");
      if(! latdir) return false;

      // grab longitude
      char *longp = strtok(NULL, ",");
      if(! longp) return false;

      // grab longitude direction
      char *longdir = strtok(NULL, ",");
      if(! longdir) return false;

      // skip speed
      tok = strtok(NULL, ",");
      if(! tok) return false;

      // skip course
      tok = strtok(NULL, ",");
      if(! tok) return false;
      
      // grab date ddmmyy
      tok = strtok(NULL, ",");
      if(! tok) return false;
      else strcpy(date_str,tok);    

      double latitude = atof(latp);
      double longitude = atof(longp);

      // convert latitude from minutes to decimal
      double degrees = floor(latitude / 100);
      double minutes = latitude - (100 * degrees);
      minutes /= 60;
      degrees += minutes;

      // turn direction into + or -
      if(latdir[0] == 'S') degrees *= -1;

      dtostrf(degrees, 9, 5, latitude_str);
      lat = degrees;

      // convert longitude from minutes to decimal
      degrees = floor(longitude / 100);
      minutes = longitude - (100 * degrees);
      minutes /= 60;
      degrees += minutes;

      // turn direction into + or -
      if(longdir[0] == 'W') degrees *= -1;

      dtostrf(degrees, 9, 5, longitude_str);
      lon = degrees;
      //-------------------------
      delay(2000);
      return fix_status;
    }
    delay(2000);
    #ifdef OLD_DEBUG
    if(DEBUG >= 1) {
      messageLCD(1000, F("Fix:"), String(fix_status)+" t:"+String(timeout));
      Serial.println(F("\tFONA GPS Waiting for GPS FIX"));
    }
    #endif
  }
  return 0;
}


boolean powerOffFONA(boolean powerOffGPS = false) {

  if( powerOffGPS ) {
    // first check if GPS is already on or off
    if(ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;OK"), 1) ) {
      #ifdef OLD_DEBUG
      if(DEBUG >= 2) {
        messageLCD(1000, F("GPS power: on"), F(">shutdown"));
        Serial.println(F("\tFONA GPS is on, -turning it off."));
      }
      #endif
      if(! ATsendReadVerifyFONA(F("AT+CGPSPWR=0"), F("OK")) )
        return false;
    }
    else {
      #ifdef OLD_DEBUG
      if(DEBUG >= 2) {
        messageLCD(1000, F("GPS power: off"), F(">OK"));
        Serial.println(F("\tFONA GPS power already is off, -skipping."));
      }
      #endif
    }
  }

  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == HIGH;
  delay(500);
  digitalWrite(FONA_POWER_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_POWER_KEY, HIGH);
  pinMode(FONA_POWER_KEY, OUTPUT);
  delay(500);
  return true;
}


void clearInitData(){
  
  strcpy(data, "IMEI=");
  strcat(data, IMEI_str);
  strcat(data, "&");
  strcat(data, "data=");

  #ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    messageLCD(2000,"DATA",">cleared");
    Serial.println(F("\t\tDATA: cleared!"));    
  }
  #endif
 
}


unsigned int saveData(){

  strcat(data, latitude_str);
  strcat(data, "#");
  strcat(data, longitude_str);
  strcat(data, "#");    
  strcat(data, date_str);
  strcat(data, "#");  
  strcat(data, time_str);
  strcat(data, "#"); 

  char batt_str[6];
  char temp_str[4];
  char hum_str[4];
 
  itoa((int)batt_voltage, batt_str, 10);
  itoa((int)temperature, temp_str, 10);
  itoa((int)humidity, hum_str, 10);
  //dtostrf(temperature, 5, 1, temp_str);

  strcat(data, batt_str);
  strcat(data, "#"); 
  strcat(data, temp_str);
  strcat(data, "#"); 
  strcat(data, hum_str);
  strcat(data, "#"); 
    
  #ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    messageLCD(2000,F("DATA"),">saved #"+String(samples));
    char index_str[4];
    itoa(strlen(data), index_str,10);
    Serial.print(F("\t\tDATA: stored, index at:"));
    Serial.print(index_str);
    Serial.print(F(", run #"));
    Serial.println(samples);
  }
  #endif   
  
  return strlen(data);
}


boolean sendDataServer(){

  // close all prevoius HTTP sessions
  ATsendReadVerifyFONA(F("AT+HTTPTERM"), F("OK"));

  // start a new HTTP session
  if(! ATsendReadVerifyFONA(F("AT+HTTPINIT"), F("OK")) )
    return false;

  // setup the HTML HEADER
  // CID = Bearer profile identifier =
  if(! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"CID\",\"1\""), F("OK")) )
    return false;

  // setup the HTML USER AGENT
  if(! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"UA\",\"CLOUDMARE1.0\""), F("OK")) )
    return false;

  // setup the HTML CONTENT
  if(! ATsendReadVerifyFONA(F("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""), F("OK")) )
    return false;

  // setup URL to send data to
  strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPPARA=\"URL\",\""));
  strcat(dataBuffer, url);
  strcat(dataBuffer, "\"");
  if(! ATsendReadVerifyFONA(dataBuffer, F("OK")) )
    return false;

  
  // crop of the last "#" in data string
  data[strlen(data)-1]='\0';

  // add final parameter, the checksum
  strcat(data, "&sum=");
  
  // add simple paritet bit att end of data string
  uint16_t sum=0;
  for(uint8_t i=0;i<strlen(data);i++)
    sum += __builtin_popcount(data[i]);   

  //#ifdef OLD_DEBUG
  if(DEBUG >= 2) {
    messageLCD(2000,"Parity SUM",String(sum));
    Serial.print(F("\t\tDATA: Parity sum of data string: "));
    Serial.println(sum);
    Serial.print(F("\t\tDATA: data="));
    Serial.println(data);
  } 
  //#endif

  char sum_str[4];
  itoa(sum,sum_str,10);
  
  strcat(data, sum_str);
  strcat(data, "&");
  
  // setup length of data to send
  strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));
  char dataLengthStr[4]; 
  itoa(strlen(data),dataLengthStr,10);
  // prepare to send data
  strcat(dataBuffer, dataLengthStr);
  strcat(dataBuffer, ",");
  strcat_P(dataBuffer, (const char PROGMEM *)F("10000"));
  if(! ATsendReadVerifyFONA(dataBuffer, F("DOWNLOAD")) )
    return false;

  // loading data
  if(! ATsendReadVerifyFONA(data, F("OK")) )
    return false;

  // sending data by HTTP POST
  if(! ATsendReadVerifyFONA(F("AT+HTTPACTION=1"), F("OK;"),1) ){ 
    ATreadFONA(0,11000);   
    if(DEBUG >= 1) {
      char* code = strtok(dataBuffer,",");
      code = strtok(NULL, ",");
      messageLCD(2000,F("HTTP"),">ERROR #"+String(code));
      Serial.print(F("\tHTTP: ERROR, code: "));
      Serial.println(code);
    }    
    return false;
  }
  else{
    ATreadFONA(0,11000); 
    if(DEBUG >= 2) {
    char* code = strtok(dataBuffer,",");
    code = strtok(NULL, ",");
    messageLCD(2000,F("HTTP"),">OK #"+String(code));
    Serial.print(F("\tHTTP: OK, code: "));
    Serial.println(strlen(code));
    }   
  }
  return true;
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
  if(watchdogActivated) {
    watchdogActivated = false;

    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
     
    if(sleepIterations >= MAX_SLEEP_ITERATIONS_GPS) {
      
      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {    
        messageLCD(1000, F("ARDUINO"), F(">booting"));
        Serial.println(F("ARDUINO awake, -booting."));
      }
      #endif

      // Reset the number of sleep iterations.
      sleepIterations = 0;

      //DO SOME WORK!
      //Fire up FONA 808 GPS and take a position reading.
      delay(3000);

        Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
      
      //readDHT11();
      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {
        messageLCD(1000, "Temp="+String(temperature), "Humid="+String(humidity));  
        Serial.print("T");
        Serial.print(temperature);
        Serial.print(" H");
        Serial.println(humidity); 
      } 
      #endif
      
      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {
        messageLCD(0, F("FONA:"), F(">power up"));
        Serial.println(F("\tFONA power up."));
      }
      #endif
      initFONA();

      getImeiFONA();
      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {
        messageLCD(0, F("FONA imei:"), IMEI_str);
        Serial.print(F("\tFONA IMEI: "));
        Serial.println(IMEI_str);
      }
      #endif
      
      if(samples==1)
        clearInitData();

      #ifdef OLD_DEBUG  
      if(DEBUG >= 1) {
        messageLCD(0, F("SDCARD:"), F(">load"));
        Serial.println(F("\tSDCARD: loading config"));
      }
      #endif
      
      loadConfigSDcard();

      #ifdef OLD_DEBUG
      if(DEBUG >= 1) {
        messageLCD(0, F("FONA:"), F(">GPS power on"));
        Serial.println(F("\tFONA GPS power on."));
      }
      #endif
      enableGpsFONA808();


      //Show battery level on display
      batteryCheckFONA();
      if(DEBUG >= 1) 
        messageLCD(1000, "Battery %", String(batt_percent) );
      


      lonAVG=0;
      latAVG=0;

     #ifdef OLD_DEBUG
      if(DEBUG == 1) {
        messageLCD(0, F("FONA:"), F(">coll. GPS data"));
        //Serial.println(F("\tFONA GPS, -collecting GPS position data."));
      }
      #endif
      
      for(int i=1;i<=GPS_AVG;i++){
        if(DEBUG >= 2) {
          messageLCD(0, F("FONA: collecting"),">smp #"+String(i) );
          //Serial.println(F("\tFONA GPS, -collecting GPS position data."));
        }
        readGpsFONA808();
        latAVG += lat;
        lonAVG += lon;   
      }
      latAVG/=GPS_AVG;
      lonAVG/=GPS_AVG;
      dtostrf(latAVG, 9, 5, latitude_str);
      dtostrf(lonAVG, 9, 5, longitude_str);
      
      if(DEBUG >= 1) {
        
        char* line1_pointer = dataBuffer;
        char* line2_pointer = dataBuffer+16;
        
        strcpy(dataBuffer,latitude_str);
        strcat(dataBuffer," ");
        strcat(dataBuffer,date_str);     
        strcat(dataBuffer,longitude_str);      
        strcat(dataBuffer," ");              
        strcat(dataBuffer,time_str);              
        messageLCD(4000, line1_pointer,line2_pointer );
       
        Serial.print(F("DATA: Lat/lon:"));
        Serial.print(latitude_str);
        Serial.print(F(" / "));
        Serial.print(longitude_str);

        Serial.print(F(" date:"));
        Serial.print(date_str);
        Serial.print(F(" time:"));
        Serial.println(time_str);
      }


      
      saveData();
     
   
      if( samples >= NUMBER_OF_DATA ){
        
        #ifdef OLD_DEBUG
        if(DEBUG >= 1) {
          messageLCD(0, F("FONA gprs:"), F(">init"));
          Serial.println(F("\tFONA gprs: initializing"));
        }
        #endif
        
        enableGprsFONA();

        sendDataServer();
          
        samples=0;
        disableGprsFONA();
      }
      samples++;


      #ifdef OLD_DEBUG
      if(DEBUG >= 2) {
        messageLCD(0, F("FONA: "), F(">power off"));
        Serial.println(F("\tFONA off."));
      }
      #endif
      // power down FONA, true=GPS aswell
      powerOffFONA(true);


      // Go to sleep!
      if(DEBUG >= 1) {
        Serial.println(F("Zzzz."));
        messageLCD(-1000, F(">Zzzz."));
      }
    }
  }

  sleep();

}

