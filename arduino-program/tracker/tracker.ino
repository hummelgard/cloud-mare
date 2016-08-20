 /*
 *
 */

#include <SdFat.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Wire.h>

//hardware-version
// #4 = BME280, LIS3dH, TMP007, FONA808, microSDcardReader, arduino pro mini


// SENSORS USED IN THE TRACKER
#define VERSION          "4.e6c0dcb" //first number hardware version, second git number
#define BME280                     // is a BME280 weather sensor used?
#define TMP007                     // is a TMP007 ir thermometer used?
#define LIS3DH                     // is a LIS3DH accelerometer used?
#define SDCARD                     // is a sdcard reader used?

// SERIAL-DEBUG / DISPLAY OPTIONS (only one may be choosen, due to memory limits
#define SERIAL_COM                 // If serial-port is being used for debugging
//#define SERIAL_LCD                 // If defined, it shows some info on the LCD display
#define SERIAL_LCD_PIN    16       // was 7 before.

// CONFIGURE SETTINGS
#define POS_SIZE           9       // Number of samples in median algorithm for GPS
#define SMP_DELTATIME      200     // delay between each GPS reading in milliseconds.
//#define GPS_WAIT         180       // Seconds to try getting a valid GPS reading

// PIN ASSIGNMENT OF ARDUINO
#define FONA_RX            8       // RX pin on arduino that connects to FONA
#define FONA_TX            9       // TX pin on arduino that connects to FONA
#define FONA_RST           4       // RESET pin on arduino that connects to FONA
#define FONA_POWER_KEY     3       // POWER pin on arduino that connects to FONA
#define FONA_PSTAT         2       // PWR STATUS pin on arduino that connects to FONA
#define SDCARD_CS         10       // pin on arduino that connects SDCARD


#define prog_char  char PROGMEM

// BME280 registers
#define BME280_REGISTER_DIG_T1       0x88
#define BME280_REGISTER_DIG_T2       0x8A
#define BME280_REGISTER_DIG_T3       0x8C

#define BME280_REGISTER_DIG_P1       0x8E
#define BME280_REGISTER_DIG_P2       0x90
#define BME280_REGISTER_DIG_P3       0x92
#define BME280_REGISTER_DIG_P4       0x94
#define BME280_REGISTER_DIG_P5       0x96
#define BME280_REGISTER_DIG_P6       0x98
#define BME280_REGISTER_DIG_P7       0x9A
#define BME280_REGISTER_DIG_P8       0x9C
#define BME280_REGISTER_DIG_P9       0x9E

#define BME280_REGISTER_DIG_H1       0xA1
#define BME280_REGISTER_DIG_H2       0xE1
#define BME280_REGISTER_DIG_H3       0xE3
#define BME280_REGISTER_DIG_H4       0xE4
#define BME280_REGISTER_DIG_H5       0xE5
#define BME280_REGISTER_DIG_H6       0xE7
 
#define BME280_REGISTER_CHIPID       0xD0
#define BME280_REGISTER_VERSION      0xD1
#define BME280_REGISTER_SOFTRESET    0xE0

#define BME280_REGISTER_CAL26        0xE1  // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID 0xF2
#define BME280_REGISTER_CONTROL      0xF4
#define BME280_REGISTER_CONFIG       0xF5
#define BME280_REGISTER_PRESSUREDATA 0xF7
#define BME280_REGISTER_TEMPDATA     0xFA
#define BME280_REGISTER_HUMIDDATA    0xFD

#define BME280_ADDRESS               0x77

// TMP007 registers
#define TMP007_TDIE                  0x01
#define TMP007_CONFIG                0x02
#define TMP007_TOBJ                  0x03
#define TMP007_STATUS                0x04
#define TMP007_STATMASK              0x05

#define TMP007_CFG_RESET             0x8000  // B1000000000000000
#define TMP007_CFG_MODEON            0x1000  // B0001000000000000

#define TMP007_CFG_1SAMPLE           0x0000
#define TMP007_CFG_2SAMPLE           0x0200  // B0000001000000000
#define TMP007_CFG_4SAMPLE           0x0400  // B0000010000000000
#define TMP007_CFG_8SAMPLE           0x0600
#define TMP007_CFG_16SAMPLE          0x0800
#define TMP007_CFG_ALERTEN           0x0100  // B0000000100000000
#define TMP007_CFG_ALERTF            0x0080
#define TMP007_CFG_TRANSC            0x0040  // B0000000001000000

#define TMP007_STAT_ALERTEN          0x8000
#define TMP007_STAT_CRTEN            0x4000

#define TMP007_DEVID                 0x1F
#define TMP007_I2CADDR               0x40

// LIS3DH registers
#define LIS3DH_REG_STATUS1           0x07
#define LIS3DH_REG_OUTADC1_L         0x08
#define LIS3DH_REG_OUTADC1_H         0x09
#define LIS3DH_REG_OUTADC2_L         0x0A
#define LIS3DH_REG_OUTADC2_H         0x0B
#define LIS3DH_REG_OUTADC3_L         0x0C
#define LIS3DH_REG_OUTADC3_H         0x0D
#define LIS3DH_REG_INTCOUNT          0x0E
#define LIS3DH_REG_WHOAMI            0x0F
#define LIS3DH_REG_TEMPCFG           0x1F
#define LIS3DH_REG_CTRL1             0x20
#define LIS3DH_REG_CTRL2             0x21
#define LIS3DH_REG_CTRL3             0x22
#define LIS3DH_REG_CTRL4             0x23
#define LIS3DH_REG_CTRL5             0x24
#define LIS3DH_REG_CTRL6             0x25
#define LIS3DH_REG_REFERENCE         0x26
#define LIS3DH_REG_STATUS2           0x27
#define LIS3DH_REG_OUT_X_L           0x28
#define LIS3DH_REG_OUT_X_H           0x29
#define LIS3DH_REG_OUT_Y_L           0x2A
#define LIS3DH_REG_OUT_Y_H           0x2B
#define LIS3DH_REG_OUT_Z_L           0x2C
#define LIS3DH_REG_OUT_Z_H           0x2D
#define LIS3DH_REG_FIFOCTRL          0x2E
#define LIS3DH_REG_FIFOSRC           0x2F
#define LIS3DH_REG_INT1CFG           0x30
#define LIS3DH_REG_INT1SRC           0x31
#define LIS3DH_REG_INT1THS           0x32
#define LIS3DH_REG_INT1DUR           0x33
#define LIS3DH_REG_CLICKCFG          0x38
#define LIS3DH_REG_CLICKSRC          0x39
#define LIS3DH_REG_CLICKTHS          0x3A
#define LIS3DH_REG_TIMELIMIT         0x3B
#define LIS3DH_REG_TIMELATENCY       0x3C
#define LIS3DH_REG_TIMEWINDOW        0x3D
#define LIS3DH_I2CADDR               0x18

float   HDOP                 = 2.0;
uint8_t GPS_FIX_MIN          = 0;
uint8_t NUMBER_OF_DATA       = 0;
uint16_t LOGGING_FREQ_SECONDS = 0;
uint16_t GPS_WAIT            = 100;

uint8_t samples = 1;
uint16_t eeprom_index = 0;

char dataBuffer[80];

// store meassured values into eeprom, which is only used for this purpose so size declaration
// dosen't matter.
char EEMEM data[75 + 45 * 10 + 12];


// USED BY: sendDataServer loadConfigSDcard
//              12345678901234567890123456789012345678901234567890
char url[50] = "0000000000000000000000000000000000000000000000000";


// USED BY: loadConfigSDcard sendDataServer enableGprsFONA
char apn[20] = "0000000000000000000";
char user[10] = "000000000";
char pwd[10] = "000000000";

float lat;
float lon;

float latArray[POS_SIZE]={0};
float lonArray[POS_SIZE]={0};
uint8_t sleepIterations = 0;
uint8_t resetCounter = 0;
volatile bool watchdogActivated = true;

uint32_t progStartTime;
uint16_t progLoopTime;

char c;

uint8_t DHTbits[5];
char sq[]="#";
boolean reset;
boolean error;
// temp variables

float float_f1;
float float_f2;
float float_f3;
float float_f4;
int int_i;
int16_t int16_i;
int16_t int16_j;
int32_t int32_i;
int32_t int32_j;
int64_t int64_i;
int64_t int64_j;
int64_t int64_k;
uint8_t uint8_i;
uint8_t uint8_j;
uint8_t uint8_k;
uint16_t uint16_i;
uint16_t uint16_j;
char str10_A[10];
char str8_B[8];
char str8_C[8];


char* char_pt1;     
char* char_pt2;       
char* bufferPointer;


// used by BME280
#ifdef BME280
 uint16_t dig_T1;
 int16_t  dig_T2;
 int16_t  dig_T3;

 uint16_t dig_P1;
 int16_t  dig_P2;
 int16_t  dig_P3;
 int16_t  dig_P4;
 int16_t  dig_P5;
 int16_t  dig_P6;
 int16_t  dig_P7;
 int16_t  dig_P8;
 int16_t  dig_P9;

 uint8_t  dig_H1;
 int16_t  dig_H2;
 uint8_t  dig_H3;
 int16_t  dig_H4;
 int16_t  dig_H5;
 int8_t   dig_H6;

 int32_t t_fine;
#endif

int I2Cadress;

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
SoftwareSerial serialLCD(17,SERIAL_LCD_PIN); // input 17 is acctually not used!


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

void write16(int addr,int data){
  Wire.beginTransmission(I2Cadress);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return;
}

void write8(byte reg, byte value)
{
 
    Wire.beginTransmission((uint8_t)I2Cadress);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();

}

uint8_t read8(byte reg)
{
  uint8_t value;

    Wire.beginTransmission((uint8_t)I2Cadress);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)I2Cadress, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

  return value;
}

uint16_t read16(byte reg)
{
  uint16_t value;


    Wire.beginTransmission((uint8_t)I2Cadress);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)I2Cadress, (byte)2);
    value = (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();


  return value;
}

uint16_t read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

int16_t readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}

/***LOW LEVEL AT FONA COMMANDS***********************************************/

uint8_t ATreadFONA(uint8_t multiline = 0, int timeout = 10000) {

  uint16_i = 0;

  while (timeout--) {
    if (uint16_i >= 254) {
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

    delay(1);
  }

  dataBuffer[uint16_i] = 0;  // null term
#ifdef SERIAL_COM   
  Serial.print(F("READ: "));
  Serial.println(dataBuffer);
#endif
  delay(100);
  return uint16_i;
  
}


void ATsendFONA(char* ATstring) {

#ifdef SERIAL_COM
  Serial.print(F("SEND: "));
  Serial.println(ATstring);
#endif
  fonaSS.println(ATstring);
  return;
}

uint8_t ATsendReadFONA(char* ATstring, uint8_t multiline = 0) {

  //messageLCD(2000, String(ATstring));
#ifdef SERIAL_COM
  Serial.print(F("SEND: "));
  Serial.println(ATstring);
#endif
  fonaSS.println(ATstring);
  return ATreadFONA(multiline);
}

uint8_t ATsendReadFONA(const __FlashStringHelper *ATstring, uint8_t multiline = 0) {

  //messageLCD(2000, String(ATstring));
#ifdef SERIAL_COM  
  Serial.print(F("SEND: "));
  Serial.println(ATstring);
#endif
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
#ifdef SERIAL_LCD  
  pinMode(SERIAL_LCD_PIN, OUTPUT);
  serialLCD.begin(9600);
  delay(500);
#endif

#ifdef SERIAL_COM
  Serial.begin(115200);
#endif
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
  progStartTime = millis();


  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated) {
    watchdogActivated = false;
    
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;

    //AWAKE, -DO SOME WORK!
    //-----------------------------------------------------------------------  
    if (sleepIterations >= LOGGING_FREQ_SECONDS/8) {
       
       if(resetCounter > 0){
         Serial.println("RESET");
         delay(100);
         softReset();
       }
       else
         resetCounter += 1;
//#ifdef SERIAL_COM
//      Serial.print("RAM:  ");Serial.print(freeRam());Serial.println(" bytes free.");
//#endif
  
      #ifdef SERIAL_LCD  
      messageLCD(1500, "HorseTracker", VERSION);
      #else
      //delay(1500);
      #endif
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      

      error = false;
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
        if (! ATsendReadVerifyFONA(F("ATE0"), F("OK")) ){
          //delay(100);
          ATreadFONA();
          //delay(100);
          if (!ATsendReadVerifyFONA(F("ATE0"), F("OK")) )//{
            //delay(100);
            //ATsendReadVerifyFONA(F("AT&W0"), F("OK"));
          //}
          //else
            reset = true; 
        }
        //delay(100);
        if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
          reset = true;
        //delay(100);
        if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
          reset = true;
        //delay(100);
        if (! ATsendReadVerifyFONA(F("AT"), F("OK")) )
          reset = true;
        //delay(100);


      } while (reset == true);
      #ifdef SERIAL_LCD
      messageLCD(1000, "FONA:", ">on");
      #else
      //delay(1000);
      #endif


      // IS THIS FIRST RUN?, -THEN INIT/CLEAR EEPROM STORAGE and LOAD SDCARD
      //-----------------------------------------------------------------------
      if (samples == 1){
        // WRITE THE HARDWARE AND SOFTWARE VERSION TO LOG
        
        // This is the start of the log, reset the index position to zero
        eeprom_index = 0;

        // Write the first log note, the IMEI number of the unit
        eeprom_write_block("ver=", &data[eeprom_index], 4);
        eeprom_index += 4;

        eeprom_write_block(VERSION, &data[eeprom_index], strlen(VERSION));
        eeprom_index += strlen(VERSION);      
        // READ IMEI NUMBER OF MODEM
        //---------------------------------------------------------------------        
        ATsendReadFONA(F("AT+GSN"), 2);
        bufferPointer = strtok(dataBuffer, ";");

        // Write the first log note, the IMEI number of the unit
        eeprom_write_block("&IMEI=", &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);
        
        #ifdef SERIAL_LCD
        // messageLCD(1000, "FONA imei:", bufferPointer);
        strcpy(str10_A, bufferPointer+11);
        #endif
       
       
        // READ IMSI NUMBER OF SIM CARD
        //---------------------------------------------------------------------        
        ATsendReadFONA(F("AT+CIMI"), 2);
        bufferPointer = strtok(dataBuffer, ";");

        // Write the first log note, the IMEI number of the unit
        eeprom_write_block("&IMSI=", &data[eeprom_index], 6);
        eeprom_index += 6;

        eeprom_write_block(bufferPointer, &data[eeprom_index], strlen(bufferPointer));
        eeprom_index += strlen(bufferPointer);
        
        #ifdef SERIAL_LCD
        strcpy(str10_A+5, bufferPointer+11);
        str10_A[4]='-';
        messageLCD(3000, "TrackerID", str10_A);
        #else
        delay(3000);
        #endif

        // LOAD USER CONFIGUARTION FROM SDCARD
        //---------------------------------------------------------------------     
#ifdef SDCARD        

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

                //Write the second log note, the name of the user
                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("t_user")) == 0 ){
                  eeprom_write_block("&user=", &data[eeprom_index], 6);
                  eeprom_index += 6;

                  eeprom_write_block(char_pt2, &data[eeprom_index], strlen(char_pt2));
                  eeprom_index += strlen(char_pt2);
                }

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("LOGGING_FREQ_SECONDS")) == 0 )
                  LOGGING_FREQ_SECONDS = atoi(char_pt2);

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("NUMBER_OF_DATA")) == 0 )
                  NUMBER_OF_DATA = atoi(char_pt2);

                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_FIX_MIN")) == 0 )
                  GPS_FIX_MIN = atoi(char_pt2);
                  
                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("HDOP")) == 0 )
                  HDOP = atof(char_pt2);
                  
                if ( strcmp_P(char_pt1, (const char PROGMEM *)F("GPS_WAIT")) == 0 )
                  GPS_WAIT = atof(char_pt2);

#else
                strcpy(apn,"online.telia.se");           
                strcpy(url,"http://mundilfare.hummelgard.com/data/add/");
                LOGGING_FREQ_SECONDS=180;
                NUMBER_OF_DATA=1;
                GPS_FIX_MIN=3;
                HDOP=1.9;
                GPS_WAIT=50;
                strcpy(str10_A, "maghum");
                char_pt2 = str10_A;               
                eeprom_write_block("&user=", &data[eeprom_index], 6);
                eeprom_index += 6;

                eeprom_write_block(char_pt2, &data[eeprom_index], strlen(char_pt2));
                eeprom_index += strlen(char_pt2);
                strcpy(str10_A, "disabled");            
                  
#endif

#ifdef SERIAL_COM
                Serial.print("SD: ");
                Serial.print(char_pt1);
                Serial.print("=");
                Serial.println(char_pt2);
#endif
#ifdef SDCARD  
              }
            }
          }
        
          SDfile.close();
        
        }
#endif  
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
        messageLCD(1000, "Battery%", bufferPointer );
        #else
        //delay(1000);
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
        

        // READ BME280 TEMP/HUMID/PRESSURE SENSOR
        //-----------------------------------------------------------------------
#ifdef BME280
        I2Cadress = BME280_ADDRESS;
        dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
        dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
        dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

        dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
        dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
        dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
        dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
        dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
        dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
        dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
        dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
        dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

        dig_H1 = read8(BME280_REGISTER_DIG_H1);
        dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
        dig_H3 = read8(BME280_REGISTER_DIG_H3);
        dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
        dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
        dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);

        write8(BME280_REGISTER_CONTROLHUMID, B101 );//0x03);
        write8(BME280_REGISTER_CONTROL, B10110101);//;0x3F);


        // READ TEMPERATURE
        int32_i = read16(BME280_REGISTER_TEMPDATA);
        int32_i <<= 8;
        int32_i |= read8(BME280_REGISTER_TEMPDATA+2);
        int32_i >>= 4;

        int32_i  = ((((int32_i>>3) - ((int32_t)dig_T1 <<1))) *
        ((int32_t)dig_T2)) >> 11;

        int32_j  = (((((int32_i>>4) - ((int32_t)dig_T1)) *
        ((int32_i>>4) - ((int32_t)dig_T1))) >> 12) *
        ((int32_t)dig_T3)) >> 14;

        t_fine = int32_i + int32_j;
        float_f1 = ((t_fine * 5 + 128) >> 8)/100.0;
        

        // READ HUMIDITY
        int32_i = read16(BME280_REGISTER_HUMIDDATA);

        int32_j = (t_fine - ((int32_t)76800));

        int32_j = (((((int32_i << 14) - (((int32_t)dig_H4) << 20) -
            (((int32_t)dig_H5) * int32_j)) + ((int32_t)16384)) >> 15) *
               (((((((int32_j * ((int32_t)dig_H6)) >> 10) *
              (((int32_j * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

        int32_j = (int32_j - (((((int32_j >> 15) * (int32_j >> 15)) >> 7) *
                ((int32_t)dig_H1)) >> 4));

        int32_j = (int32_j < 0) ? 0 : int32_j;
        int32_j = (int32_j > 419430400) ? 419430400 : int32_j;
        float_f2 = (int32_j>>12)/1024.0;

   
        // READ PRESSURE         
        int32_i = read16(BME280_REGISTER_PRESSUREDATA);
        int32_i <<= 8;

        int32_i |= read8(BME280_REGISTER_PRESSUREDATA+2);
        int32_i >>= 4;

        int64_i = ((int64_t)t_fine) - 128000;
        int64_j = int64_i * int64_i * (int64_t)dig_P6;
        int64_j = int64_j + ((int64_i*(int64_t)dig_P5)<<17);
        int64_j = int64_j + (((int64_t)dig_P4)<<35);
        int64_i = ((int64_i * int64_i * (int64_t)dig_P3)>>8) +
          ((int64_i * (int64_t)dig_P2)<<12);
        int64_i = (((((int64_t)1)<<47)+int64_i))*((int64_t)dig_P1)>>33;

        int64_k = 1048576 - int32_i;
        int64_k = (((int64_k<<31) - int64_j)*3125) / int64_i;
        int64_i = (((int64_t)dig_P9) * (int64_k>>13) * (int64_k>>13)) >> 25;
        int64_j = (((int64_t)dig_P8) * int64_k) >> 19;

        int64_k = ((int64_k + int64_i + int64_j) >> 8) + (((int64_t)dig_P7)<<4);
        float_f3 = (float)int64_k/256;


        // Write BME280 temp data to log
        dtostrf(float_f1, 1, 2, str10_A);
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
        
        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1; 


         // Write BME280 humidity data to log
        dtostrf(float_f2, 1, 2, str10_A);
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
        
        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1; 

                 
         // Write BME280 pressure data to log
        dtostrf(float_f3, 1, 2, str10_A);
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
        
        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;         
#endif


        // READ TMP007 IR THERMOMETER SENSOR
        //-----------------------------------------------------------------------
#ifdef TMP007
        I2Cadress = TMP007_I2CADDR;
        write16(TMP007_CONFIG, TMP007_CFG_MODEON | TMP007_CFG_ALERTEN | 
                TMP007_CFG_TRANSC | TMP007_CFG_1SAMPLE);
        write16(TMP007_STATMASK, TMP007_STAT_ALERTEN |TMP007_STAT_CRTEN);
        int16_i = read16(TMP007_TOBJ);

        write16(TMP007_CONFIG, 0x0);
        int16_i >>=2;

        float_f1 = int16_i;
        float_f1 *= 0.03125; // convert to celsius      

         // Write TMP007 temp data to log
        dtostrf(float_f1, 1, 2, str10_A);
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);
        
        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;    

#endif
        // READ LIS3DH ACCELEROMETER SENSOR
        //-----------------------------------------------------------------------
#ifdef LIS3DH
        I2Cadress = LIS3DH_I2CADDR;
        // enable all axes, at 10 HZ, and use normal-power moder
        write8(LIS3DH_REG_CTRL1, 0b00100111);

        // High res & BDU enabled
        write8(LIS3DH_REG_CTRL4, 0b10001000);
        

        // dividers 2g = 16380, 4g = 8190, 8g = 4096, 16g = 1365

        // read acceleration in X-direction
        int16_i = read8(LIS3DH_REG_OUT_X_L);
        int16_j = read8(LIS3DH_REG_OUT_X_H) << 8;
        int16_i |= int16_j;
        itoa(int16_i, str10_A, 10);  

        #ifdef SERIAL_COM
        //  Serial.print(F("LIS3DH-X: "));
        //  Serial.println(str10_A);
        #endif
        // write X acceleration to log 
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // read acceleration in Y-direction
        int16_i = read8(LIS3DH_REG_OUT_Y_L);
        int16_j = read8(LIS3DH_REG_OUT_Y_H) << 8;
        int16_i |= int16_j;
        itoa(int16_i, str10_A, 10);  

        #ifdef SERIAL_COM
        //Serial.print(F("LIS3DH-Y: "));
        //Serial.println(str10_A);
        #endif
        // write Y acceleration to log 
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // read acceleration in Z-direction
        int16_i = read8(LIS3DH_REG_OUT_Z_L);
        int16_j = read8(LIS3DH_REG_OUT_Z_H) << 8;
        int16_i |= int16_j;
        itoa(int16_i, str10_A, 10);  


        #ifdef SERIAL_COM
        //Serial.print(F("LIS3DH-Z: "));
        //Serial.println(str10_A);
        #endif
        // write Z acceleration to log 
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;
        
        // power down, verkar ge lagging update på accelerationen
        //write8(LIS3DH_REG_CTRL1, 0b00000000);
#endif


        // TURN ON THE GPS UNIT IN FONA MODULE
        //-----------------------------------------------------------------------
        #ifdef SERIAL_LCD
        messageLCD(1000,"FONA-gps", ">on");
        #else
        //delay(1000);
        #endif
        // first check if GPS is  on or off, if off, -turn it on
        //if( ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 0;;OK"), 2) )
        ATsendReadFONA(F("AT+CGPSPWR=1"));
  

        //READ GPS LOCATION DATA of the FONA 808 UNIT
        //-----------------------------------------------------------------------  


        for (uint8_j = 0; uint8_j < POS_SIZE; uint8_j) {
          #ifdef SERIAL_LCD
          strcpy(str10_A, "get# ");
         itoa(uint8_j+1,str10_A,10);
          messageLCD(0, "FONA-gps", str10_A );
          #endif
          int_i = GPS_WAIT;    
          while (int_i) {
            delay(SMP_DELTATIME); 
            // DO WE HAVE A BASIC GPS FIX?
            if ( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("+CGPSSTATUS: Location Not Fix;;OK"), 2) )
              uint8_k = 1;
            else if (dataBuffer[22] == '3')
              uint8_k = 3;
            else if (dataBuffer[22] == '2')
              uint8_k = 2;
            else if (dataBuffer[22] == 'U')
              uint8_k = 0;


            // IS FIX GOOD ENOUGH?         
            if (uint8_k >= GPS_FIX_MIN) {
              delay(SMP_DELTATIME); 
              ATsendReadFONA(F("AT+CGPSINF=8"), 2);   // Check hdop value, high = bad accuarancy  
              if(uint8_j == 0){
              
              }
              char_pt1 = dataBuffer;
              for(uint8_i=0; uint8_i<16; uint8_i++)         // hdop is at slot 16 in string, separated by ','
                char_pt1 = strchr(char_pt1+1, ',');    
                                                          
              char_pt1++;                             // add one so we pass by the last ','
              if(char_pt1[5]==',') 
                strncpy(str10_A, char_pt1, 5);
              if(char_pt1[4]==',') 
                strncpy(str10_A, char_pt1, 4);
                 
              float_f1=atof(str10_A);                 // grab the hdop value
              
              #ifdef SERIAL_LCD
              messageLCD(0, "FONA-hdop", str10_A );
              #endif
              
              // IS HDOP GOOD ENOUGH?  0.78 is the lowest I seen so far!
              if(float_f1 <= HDOP){     
                delay(SMP_DELTATIME);                         
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
                uint8_j++;

                break;     
              }    
      
            }  
            else{
              delay(1000-SMP_DELTATIME);
            }
            
            if(int_i>0) 
              int_i--;
            else 
              break;
          }

          if (int_i == 0){
            error = true;
            break;
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
        
        //0123 4 5678
        //01234 5 67890
        //012345 6 7 8901234
        //0123456789 0 1234567890      

        // WRTIE LATITUDE GPS DATA TO LOG
        dtostrf(latArray[4], 1, 5, str10_A);

        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        eeprom_write_block(sq, &data[eeprom_index], 1);
        eeprom_index += 1;

        // WRTIE LONGITUDE GPS DATA TO LOG
        dtostrf(lonArray[4], 1, 5, str10_A);
        
        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

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
        if ( samples >= NUMBER_OF_DATA && error == false) {



        // TURN ON GPRS
        //-----------------------------------------------------------------------         
        //ATsendReadFONA(F("AT+CIPSHUT"));

        //ATsendReadFONA(F("AT+CGATT?"), 2);

        //ATsendReadFONA(F("AT+CGATT=0"));

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
        messageLCD(1000, "FONA-gprs", ">on");
        #else
        
        #endif
delay(1000);

        // SENDING DATA BY HTTP POST
        //-----------------------------------------------------------------------         

        // close all prevoius HTTP sessions
        //ATsendReadFONA(F("AT+HTTPTERM"));

        // start a new HTTP session
        ATsendReadFONA(F("AT+HTTPINIT")); 

        // setup the HTML HEADER
        // CID = Bearer profile identifier =
        ATsendReadFONA(F("AT+HTTPPARA=\"CID\",\"1\""));
  
        // setup the HTML USER AGENT
        ATsendReadFONA(F("AT+HTTPPARA=\"UA\",\"MUNDILFARE1.1\""));
  
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
        for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
          eeprom_read_block(str10_A, &data[uint16_i], 1);
          uint16_j += __builtin_popcount(str10_A[0]);
        }

        itoa(uint16_j, str10_A, 10);
        strcat_P(str10_A, (const char PROGMEM *)F("&"));

        eeprom_write_block(str10_A, &data[eeprom_index], strlen(str10_A));
        eeprom_index += strlen(str10_A);

        
        for (uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
          eeprom_read_block(str10_A, &data[uint16_i], 1);
          Serial.write(*str10_A);
        }
        Serial.write('\n');
        
        // setup length of data to send
        strcpy_P(dataBuffer, (const char PROGMEM *)F("AT+HTTPDATA="));

        itoa(eeprom_index + 1, str10_A, 10);
        strcat(dataBuffer, str10_A);

        strcat_P(dataBuffer, (const char PROGMEM *)F(","));
          strcat_P(dataBuffer, (const char PROGMEM *)F("10000"));
          ATsendReadFONA(dataBuffer, 0);

          // downloading data to send to FONA
          for ( uint16_i = 0; uint16_i < eeprom_index; uint16_i++) {
            eeprom_read_block(str10_A, &data[uint16_i], 1);
            fonaSS.write(str10_A[0]);
            //Serial.write(str10_A[0]);
          }
          fonaSS.write('\n');

          // Check if download was OK? (-eat up OK)
          ATreadFONA();

          // sending data by HTTP POST
          ATsendReadFONA(F("AT+HTTPACTION=1"));

          // read reply from server, HTTP code
          ATreadFONA(0, 11000);
          bufferPointer = strtok(dataBuffer, ",");
          bufferPointer = strtok(NULL, ","); 
          
          #ifdef SERIAL_LCD
          
          if ( strncmp(bufferPointer,"200",3)==0 )
            messageLCD(1000,"HTTP OK", bufferPointer );
          else
            messageLCD(1000, "HTTP ERR", bufferPointer );
          #else
          //delay(1000);
          #endif
          



          // TURN OFF GPRS
          //-----------------------------------------------------------------------
          
          ATsendReadFONA(F("AT+SAPBR=0,1"));
          
          ATsendReadFONA(F("AT+CIPSHUT"));
      
          ATsendReadFONA(F("AT+CGATT=0"));

          samples=0;
        }
        samples++;

        // POWER DOWN FONA
        //-----------------------------------------------------------------------  

        // first check if GPS is already on or off, if so, -shut it down!
        //if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;;OK"), 2) ) {
        ATsendReadFONA(F("AT+CGPSPWR=0"));

        if (!ATsendReadVerifyFONA(F("AT+CPOWD=1"), F("NORMAL POWER DOWN")) ){
          // Check if FONA is ON, if not turn it off!
          delay(1000);
          if (digitalRead(FONA_PSTAT) == true ) {
            pinMode(FONA_POWER_KEY, OUTPUT);
            digitalWrite(FONA_POWER_KEY, HIGH);
            delay(100);
            digitalWrite(FONA_POWER_KEY, LOW);
            delay(2000);
            digitalWrite(FONA_POWER_KEY, HIGH);
            delay(100);
          }    

        }
        //ATsendReadFONA(F("AT+CPOWD=1"));
        delay(100);


      
        if (error == true)
          samples=1;
        
        #ifdef SERIAL_LCD
        messageLCD(-1000, "ARDUINO", ">sleep");
        #else
        //delay(1000);
        #endif
       
        // reset the wdt-timer, so it dosen't trigger falty when enter sleep.
        wdt_reset(); 

        // how long did this run take, subtract that from loop intervall of sleep.
        // when sleept long enought, SDcard is read and LOGGING_FREQ_SECONDS is reset.

        progLoopTime = (millis() - progStartTime)/1000;
        if( progLoopTime < LOGGING_FREQ_SECONDS - 20 )
           LOGGING_FREQ_SECONDS = LOGGING_FREQ_SECONDS - progLoopTime;

      }

      sleep();
    }


}

