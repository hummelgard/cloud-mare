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
#include <avr/pgmspace.h>
#include "MemoryFree.h"

#define prog_char  char PROGMEM

//#define MAX_SLEEP_ITERATIONS_GPS   LOGGING_FREQ_SECONDS / 8

#define DHT11_pin       15
#define FONA_RX         8
#define FONA_TX         9
#define FONA_RST        2
#define FONA_POWER_KEY  5
#define FONA_PSTAT      4
#define GPS_WAIT        200
#define SDCARD_CS       10
#define SERIAL_LCD      true
#define POS_SIZE        19

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

uint8_t samples = 1;
uint8_t dataDHT11[6] = {0};
uint16_t eeprom_index = 0;


char dataBuffer[80];

char EEMEM data[26 + 45 * 10 + 12];
//char data[26+45*1+12] = {0};

// USED BY: sendDataServer loadConfigSDcard
//12345678901234567890123456789012345678901234567890
char url[45] = "00000000000000000000000000000000000000000000";
//char url[] ="http://cloud-mare.hummelgard.com:88/addData";


// USED BY: loadConfigSDcard sendDataServer enableGprsFONA
char apn[20] = "0000000000000000000";
char user[10] = "000000000";
char pwd[10] = "000000000";


float lat;
float lon;
//float latAVG;
//float lonAVG;
float latArray[POS_SIZE]={0};
float lonArray[POS_SIZE]={0};
uint8_t sleepIterations = 0;
volatile bool watchdogActivated = true;
int MPU9150_I2C_ADDRESS = 0x68;

//uint32_t cycles[60];
byte L;
byte H;
char c;
char sq[]="#";
boolean reset;
// temp variables

float float_f1;
float float_f2;
float float_f3;
float float_f4;
int int_i;
int16_t int16_i;
uint8_t uint8_i;
uint8_t uint8_j;
uint8_t uint8_k;
uint16_t uint16_i;
uint16_t uint16_j;
uint32_t uint32_i;
char str8_A[8];
char str8_B[8];
char str8_C[8];


char* char_pt1;     
char* char_pt2;       
char* bufferPointer;


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


int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/***SERIAL DISPLAY **********************************************************/

void messageLCD(const int time, const char* line1, const char* line2 = "") {

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

int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  H = Wire.read();

  return (int16_t)((H<<8)+L);
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
      c =  fonaSS.read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (uint16_i == 0)   // the first 0x0A is ignored
          continue;

        if (!multiline--) {
          timeout = 0;         // the second 0x0A is the end of the line
          break;
        }


      }
      if (c == 0xA)
        dataBuffer[uint16_i] = ';';
      else
        dataBuffer[uint16_i] = c;
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
  Serial.println(ATstring);

  fonaSS.println(ATstring);
  return;
}

uint8_t ATsendReadFONA(char* ATstring, uint8_t multiline = 0) {

  //messageLCD(2000, String(ATstring));
  Serial.print(F("SEND: "));
  Serial.println(ATstring);

  fonaSS.println(ATstring);
  return ATreadFONA(multiline);
}

uint8_t ATsendReadFONA(const __FlashStringHelper *ATstring, uint8_t multiline = 0) {

  //messageLCD(2000, String(ATstring));
  Serial.print(F("SEND: "));
  Serial.println(ATstring);

  fonaSS.println(ATstring);
  return ATreadFONA(multiline);
}


boolean ATsendReadVerifyFONA(char* ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0) {

  if (ATsendReadFONA(ATstring, multiline)) {
    if ( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}


boolean ATsendReadVerifyFONA(const __FlashStringHelper *ATstring, const __FlashStringHelper *ATverify, uint8_t multiline = 0) {
  if (ATsendReadFONA(ATstring, multiline)) {
    if ( strcmp_P(dataBuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;
  }
}



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

void softReset(){
asm volatile ("  jmp 0");
}
//softReset(); // to make soft reset, put it in code were you want it!

void(* resetFunc) (void) = 0; //declare reset function @ address 0

//LOOP
//-------------------------------------------------------------------------------------------
void loop() {



  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated) {
    watchdogActivated = false;

    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;

    if (sleepIterations >= LOGGING_FREQ_SECONDS) {
    Serial.print("RAM:");Serial.println(freeRam());
      //AWAKE, -DO SOME WORK!
      //-----------------------------------------------------------------------    
      #ifdef SERIAL_LCD  
      messageLCD(1000, "ARDUINO", ">booting");
      #endif
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      


      // START UP FONA 808 MODULE
      //-----------------------------------------------------------------------
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

      reset = false;
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

      } while (reset == true);
      #ifdef SERIAL_LCD
      messageLCD(0, "FONA:", ">on");
      #endif


      // IS THIS FIRST RUN?, -THEN INIT/CLEAR EEPROM STORAGE and LOAD SDCARD
      //-----------------------------------------------------------------------
      if (samples == 1){
      
        // READ IMEI NUMBER OF SIM CARD
        //---------------------------------------------------------------------        
        ATsendReadFONA(F("AT+GSN"), 2);
        bufferPointer = strtok(dataBuffer, ";");

        // This is the start of the log, reset the index position to zero
        eeprom_index = 0;

        // Write the first log note, the IMEI number of the unit
        eeprom_write_block("IMEI=", &data[eeprom_index], 5);
        eeprom_index += 5;

        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);
        
        #ifdef SERIAL_LCD
        messageLCD(500, "FONA imei:", bufferPointer);
        #endif


        // LOAD USER CONFIGUARTION FROM SDCARD
        //---------------------------------------------------------------------     
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

                //Write the second log note, the name of the horse
                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("name")) == 0 ){
                  eeprom_write_block("&name=", &data[eeprom_index], 6);
                  eeprom_index += 6;

                  eeprom_write_block(char_pt2, &data[eeprom_index], strlen(char_pt2));
                  eeprom_index += strlen(char_pt2);
                }

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
                  LOGGING_FREQ_SECONDS = atoi(char_pt2)/8;

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
                  GPS_FIX_MIN = atoi(char_pt2);

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_AVG")) == 0 )
                  GPS_AVG = atoi(char_pt2);

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
                  NUMBER_OF_DATA = atoi(char_pt2);



                Serial.print("SD: ");
                Serial.print(char_pt1);
                Serial.print("=");
                Serial.println(char_pt2);
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
        bufferPointer = strtok(dataBuffer, ":");
        // skip charge state of battery
        bufferPointer = strtok(NULL, ",");
        

        // read charge percent of battery
        bufferPointer = strtok(NULL, ",");

        #ifdef SERIAL_LCD
        messageLCD(500, "Battery%", bufferPointer );
        #endif
        // save charge percent to log
        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // read battery millivoltage
        bufferPointer = strtok(NULL, ";");
  
        // save battery voltage to log
        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        

/*
        // READ DATA FROM TEMP/HUMID SENSOR DHT11
        //-----------------------------------------------------------------------
        #ifdef SERIAL_LCD
        messageLCD(500, "DHT11", ">temp/hygr" );
        #endif
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

        //uint32_t* cycles = (uint32_t*) dataBuffer;
        //uint32_t cycles[80];
        Serial.print("freeDHT11:");Serial.println(freeRam());
        // uint32_t* cycles = (uint32_t*) malloc(80*sizeof(uint32_t));
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
        //free(cycles);
*/
        // Write DHT11 data to log
        itoa(dataDHT11[0], str8_A, 10);
        
        eeprom_write_block(str8_A, &data[eeprom_index], 2);
        eeprom_index += 2;


        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;         
          
        itoa(dataDHT11[2], str8_A, 10);
       
        eeprom_write_block(str8_A, &data[eeprom_index], 2);
        eeprom_index += 2;

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // READ DATA FROM ACCELEROMETER MPU9150
        //-----------------------------------------------------------------------    
        // INIT MPU-9150  
        #ifdef SERIAL_LCD
        messageLCD(500, "MPU9150", ">init" );
        #endif
        MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B00000010);     //Wake up MPU
        MPU9150_writeSensor(MPU9150_SMPLRT_DIV, B00000001);     //Set sample rate divider
        MPU9150_writeSensor(MPU9150_CONFIG, 6);                 //Set lowpass filter to 5Hz
        MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, B00000000);   //Set accelerometer range to +/-2g
        MPU9150_writeSensor(MPU9150_USER_CTRL, B00000000);      //Disable MPU as master for I2C slave
        MPU9150_writeSensor(MPU9150_INT_PIN_CFG, B00110010);    //Set bypass mode on I2C slave
        MPU9150_writeSensor(MPU9150_PWR_MGMT_2, B00000111);     //Put xyz gyros to standby
        
        // READ MPU-9150 TEMP DATA 
        #ifdef SERIAL_LCD
        messageLCD(500, "MPU9150", ">temp" );
        #endif               
        MPU9150_I2C_ADDRESS = 0x68;
        delay(100);
        int16_i = MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H)/340+36.53;
        dtostrf(int16_i, 5, 1, str8_A);
  
        // WRITE MPU TEMP TO LOG
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // READ MPU-9150 ACCELERATION DATA
        #ifdef SERIAL_LCD
        messageLCD(500, "MPU9150", ">accel." );
        #endif
        // read acceleration in X-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
        itoa(int16_i, str8_A, 10);

        // write X acceleration to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // read acceleration in Y-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
        itoa(int16_i, str8_A, 10);
        
        // write Y acceleration to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // read acceleration in Z-direction
        int16_i = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
        itoa(int16_i, str8_A, 10);
        
        // write Z acceleration to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        #ifdef SERIAL_LCD
        messageLCD(500, "MPU9150", ">magneto" );
        #endif
        // READ MPU-9150 MAGNETO/COMPASS DATA
        MPU9150_I2C_ADDRESS = 0x0c;
  
        MPU9150_writeSensor(0x0A, 0x02);
        delay(100);

        // read compass data in X direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
        itoa(int16_i, str8_A, 10);

        // write compass X to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
  
        // read compass data in Y direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
        itoa(int16_i, str8_A, 10);

        // write compass Y to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // read compass data in Z direction
        int16_i = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
        itoa(int16_i, str8_A, 10);        
        
        // write compass Z to log
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;


        // SLEEP MPU-9150
        delay(100);
        #ifdef SERIAL_LCD
        messageLCD(500, "MPU9150", ">sleep" );
        #endif
        MPU9150_I2C_ADDRESS = 0x0C; 
        MPU9150_writeSensor(0x0A, B00000000);
        MPU9150_I2C_ADDRESS = 0x68; 
        MPU9150_writeSensor(MPU9150_PWR_MGMT_1, B01000000);
        


        // TURN ON THE GPS UNIT IN FONA MODULE
        //-----------------------------------------------------------------------
        #ifdef SERIAL_LCD
        messageLCD(0,"FONA-gps", ">on");
        #endif
        // first check if GPS is  on or off, if off, -turn it on
        if( ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 0;;OK"), 2) )
          ATsendReadFONA(F("AT+CGPSPWR=1"));
  


        //READ GPS LOCATION DATA of the FONA 808 UNIT
        //-----------------------------------------------------------------------  

        // uint8_k  fix_status
        // uint8_j  counter
        // uint8_i  timeout
        for (uint8_j = 0; uint8_j < POS_SIZE; uint8_j) {
Serial.print("freeGPS:");Serial.println(freeRam());
          #ifdef SERIAL_LCD
          strcpy(str8_A, "get# ");
          itoa(uint8_j+1,str8_A,10);
          messageLCD(0, "FONA-gps", str8_A );
          #endif
          uint8_i = GPS_WAIT;    
          while (uint8_i--) {

            if ( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("+CGPSSTATUS: Location Not Fix;;OK"), 2) )
              uint8_k = 1;
            else if (dataBuffer[22] == '3')
              uint8_k = 3;
            else if (dataBuffer[22] == '2')
              uint8_k = 2;
            else if (dataBuffer[22] == 'U')
              uint8_k = 0;

            if (uint8_k >= GPS_FIX_MIN) {

              ATsendReadFONA(F("AT+CGPSINF=32"), 2);

              // skip mode
              bufferPointer = strtok(dataBuffer, ",");

              // grab current UTC time hhmmss.sss ,-skip the last three digits.
              bufferPointer = strtok(NULL, ",");
              strncpy(str8_B, bufferPointer, 6);

              // skip valid fix
              bufferPointer = strtok(NULL, ",");

              // grab the latitude
              char_pt1 = strtok(NULL, ",");
              float_f1 = atof(char_pt1);

              // grab latitude direction
              char_pt1 = strtok(NULL, ",");
              
              // turn direction into + or -
              if (char_pt1[0] == 'S') float_f3 *= -1;
              
              // grab longitude
              char_pt1 = strtok(NULL, ",");
              float_f2 = atof(char_pt1);

              // grab longitude direction
              char_pt1 = strtok(NULL, ",");
              
              // turn direction into + or -
              if (char_pt1[0] == 'W') float_f3 *= -1;



              // skip speed
              bufferPointer = strtok(NULL, ",");

              // skip course
              bufferPointer = strtok(NULL, ",");

              // grab date ddmmyy
              bufferPointer = strtok(NULL, ",");
              strcpy(str8_C, bufferPointer);

              //float_f1 latitude
              //float_f2 longitude
              //float_f3 degrees
              //float_f4 minutes
              
              // convert latitude from minutes to decimal
              float_f3 = floor(float_f1 / 100);
              float_f4 = float_f1 - (100 * float_f3);
              float_f4 /= 60;
              float_f3 += float_f4;

              

              lat = float_f3;

              // convert longitude from minutes to decimal
              float_f3 = floor(float_f2 / 100);
              float_f4 = float_f2 - (100 * float_f3);
              float_f4 /= 60;
              float_f3 += float_f4;



              lon = float_f3;

              latArray[uint8_j]=lat;
              lonArray[uint8_j]=lon;
              delay(4000);        
              uint8_j++;

              break;     
            }           
            else
              delay(1000);
          }
        }

        
        //CALCULATE THE MEDIAN VALUE OF THE LOCATION DATA
        //-----------------------------------------------------------------------          
        for (uint8_i = 0; uint8_i < POS_SIZE; uint8_i++)  
          for (uint8_j = 1; uint8_j < POS_SIZE-uint8_i; uint8_j++) {
            if( latArray[uint8_j]<=latArray[uint8_j-1] ){
              float_f1 = latArray[uint8_j];
              latArray[uint8_j] = latArray[uint8_j-1];
              latArray[uint8_j-1] = float_f1;
            }
            if( lonArray[uint8_j]<=lonArray[uint8_j-1] ){
              float_f1 = lonArray[uint8_j];
              lonArray[uint8_j] = lonArray[uint8_j-1];
              lonArray[uint8_j-1] = float_f1;
            }

          }
           
        //01234 5 67890
        //012345 6 7 890123 4
        //0123456789 0 1234567890      

        // WRTIE LATITUDE GPS DATA TO LOG
        dtostrf((latArray[int(POS_SIZE/2)+1]+latArray[int(POS_SIZE/2)]+latArray[int(POS_SIZE/2)-1])/3, 9, 5, str8_A);

        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // WRTIE LONGITUDE GPS DATA TO LOG
        dtostrf((lonArray[int(POS_SIZE/2)-1]+lonArray[int(POS_SIZE/2)]+lonArray[int(POS_SIZE/2)+1])/3, 9, 5, str8_A);
        
        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // WRTIE GPS DATE TO LOG
        eeprom_write_block(str8_B, &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // WRTIE GPS TIME TO LOG
        eeprom_write_block(str8_C, &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(sq, &data[eeprom_index], 1);
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
         

        strcpy_P(dataBuffer, (const char PROGMEM *) F("AT+SAPBR=3,1,\"APN\",\""));

        strcat(dataBuffer, apn);
        strcat(dataBuffer, "\"");
        ATsendReadFONA(dataBuffer);

        if (user == "") {
          strcpy_P(dataBuffer, (const char PROGMEM *) F("AT+SAPBR=3,1,\"USER\",\""));
          strcat(dataBuffer, user);
          strcat_P(dataBuffer, (const char PROGMEM *) F("\""));
          ATsendReadFONA(dataBuffer);
        }

        if (pwd == "") {
          strcpy_P(dataBuffer,(const char PROGMEM *) F("AT+SAPBR=3,1,\"PWD\",\""));
          strcat(dataBuffer, pwd);
          strcat_P(dataBuffer, (const char PROGMEM *)F("\""));
          ATsendReadFONA(dataBuffer);
        }

        ATsendReadFONA(F("AT+SAPBR=1,1"));
        #ifdef SERIAL_LCD
        messageLCD(0, "FONA-gprs", ">on");
        #endif


        // SENDING DATA BY HTTP POST
        //-----------------------------------------------------------------------         

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
        strcat_P(dataBuffer, (const char PROGMEM *)F("\""));
        ATsendReadFONA(dataBuffer);
    
        // crop of the last "#" in data string
        eeprom_index--;

        // add final parameter, -the bit-checksum 
        eeprom_write_block("&sum=", &data[eeprom_index], 5);
        eeprom_index += 5;

        uint16_j = 0;
        //char pop[1];
        for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
          eeprom_read_block(str8_A, &data[uint16_i], 1);
          uint16_j += __builtin_popcount(str8_A[0]);
        }

        itoa(uint16_j, str8_A, 10);
        strcat_P(str8_A, (const char PROGMEM *)F("&"));

        eeprom_write_block(str8_A, &data[eeprom_index], strlen(str8_A));
        eeprom_index += strlen(str8_A);

        
        for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
          eeprom_read_block(str8_A, &data[uint16_i], 1);
          Serial.write(*str8_A);
        }
        Serial.write('\n');
        
        // setup length of data to send
        strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));

        itoa(eeprom_index + 1, str8_A, 10);
        strcat(dataBuffer, str8_A);

        strcat_P(dataBuffer, (const char PROGMEM *)F(","));
          strcat_P(dataBuffer, (const char PROGMEM *)F("10000"));
          ATsendReadFONA(dataBuffer, 0);

          // downloading data to send to FONA
          for ( uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
            eeprom_read_block(str8_A, &data[uint16_i], 1);
            fonaSS.write(str8_A[0]);
            //Serial.write(str8_A[0]);
          }
          fonaSS.write('\n');
          //Serial.write('\n');

          // Check if download was OK? (-eat up OK)
          ATreadFONA();

          // sending data by HTTP POST
          Serial.print("freeHTTPsend:");Serial.println(freeRam());
          ATsendReadFONA(F("AT+HTTPACTION=1"));

          // read reply from server, HTTP code
          ATreadFONA(0, 11000);
          bufferPointer = strtok(dataBuffer, ",");
          bufferPointer = strtok(NULL, ","); 
          #ifdef SERIAL_LCD
          if ( strncmp(bufferPointer,"302",3)==0 )
            messageLCD(500,"HTTP OK", bufferPointer );
          else
            messageLCD(500, "HTTP ERR", bufferPointer );
          #endif
          



          // TURN OFF GPRS
          //-----------------------------------------------------------------------
          ATsendReadFONA(F("AT+CIPSHUT"));

          ATsendReadFONA(F("AT+CGATT?"), 2);

          samples=0;
        }
        samples++;

        // POWER DOWN FONA
        //-----------------------------------------------------------------------  

        // first check if GPS is already on or off, if so, -shut it down!
        if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;;OK"), 2) ) {
          ATsendReadFONA(F("AT+CGPSPWR=0"));
        }
        ATsendReadFONA(F("AT+CPOWD=1"));

/*
        if ( samples >= NUMBER_OF_DATA ) {
          samples=1;
           #ifdef SERIAL_LCD
          messageLCD(-500, "ARDUINO", ">sleep");
          #endif
          sleep();
          #ifdef SERIAL_LCD
           messageLCD(-500, "ARDUINO", ">Reset");
          #endif
          delay(100);
          softReset();
        }
        else{
          samples++;    
        // GO TO SLEEP
        #ifdef SERIAL_LCD
        messageLCD(-1000, "ARDUINO", ">sleep");
        #endif
        }
*/

      }
    }
    #ifdef SERIAL_LCD
    messageLCD(-1000, "ARDUINO", ">sleep");
    #endif
    sleep();
    //delay(5000);
    //watchdogActivated = true;

}

