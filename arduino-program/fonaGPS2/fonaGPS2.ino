/**
 *  ___ ___  _  _   _     ___  __  ___    ___ ___  ___
 * | __/ _ \| \| | /_\   ( _ )/  \( _ )  / __| _ \/ __|
 * | _| (_) | .` |/ _ \  / _ \ () / _ \ | (_ |  _/\__ \
 * |_| \___/|_|\_/_/ \_\ \___/\__/\___/  \___|_|  |___/
 *
 * This example is meant to work with the Adafruit
 * FONA 808 Shield or Breakout.
 *
 * Copyright: 2015 Adafruit
 * Author: Todd Treece
 * Licence: MIT
 *
 */
#include "Adafruit_FONA.h"
#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

float data[500];


#define GSM_ONLY true

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   60       // Seconds to wait before a new sensor reading is logged.
                                                   
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
                                                         // a sensor reading is taken and sent to the server.
                                                         // Don't change this unless you also change the 
                                                         // watchdog timer configuration.


#define FONA_POWER_KEY 5
                                                     
// standard pins for the 808 shield
#define FONA_RX 8
#define FONA_TX 9
#define FONA_RST 2



// This is to handle the absence of software serial on platforms
// like the Arduino Due. Modify this code if you are using different
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

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

void setup() {
  pinMode(FONA_POWER_KEY, INPUT);

   // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
  Serial.println(F("Setup complete."));    
}

int sleepIterations = 0;
volatile bool watchdogActivated = false;

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
  Serial.println(F("zzzz..."));
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
  Serial.println(F("Awake!..."));
}


void read_FONA_GPS(float &latGPS, float &lonGPS,float &latGSM, float &lonGSM, boolean &mode, String &IMEI_id, uint16_t &batteryLevel){

  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  IMEI_id=imei;
  if (boolean gps_success = fona.getGPS(&latGPS, &lonGPS)) {
    mode = !GSM_ONLY;
  }
  Serial.print("gps_status: ");
  boolean gsmloc_success = fona.getGSMLoc(&lonGSM, &latGSM);
  fona.getBattPercent(&batteryLevel); 
  //dlat = (lat1 - latGPS) / 180 * 3.14159;
  //dlon = (lon1 - lonGPS) / 180 * 3.14159;
  //a =sin(dlat/2) * sin(dlat/2) + cos(lat1/180*3.14159) * cos(latGPS/180*3.14159) * sin(dlon/2) * sin(dlon/2); 
  //c = 2* atan2(sqrt(a),sqrt(1-a));
  //distance= R * c * 1000;
  

}

void printLCD(float &latGPS, float &lonGPS,float &latGSM, float &lonGSM, const boolean mode, int &page,  uint16_t &batteryLevel){
    if (page==1) {
  serialLCD.write(254); 
  serialLCD.write(128);   
  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  serialLCD.write(254); 
  serialLCD.write(128);
  
  serialLCD.write(254); 
  serialLCD.write(128+13);
  if (mode==GSM_ONLY){
    serialLCD.write("G"); 
  }
  serialLCD.write(254); 
  serialLCD.write(128+14);
  serialLCD.print(batteryLevel); 
  serialLCD.write(254); 
  serialLCD.write(192);
  serialLCD.write("G");
  serialLCD.print(latGSM,4);
  serialLCD.write("/"); 
  serialLCD.print(lonGSM,4);

  
  delay(1000);
  }
        
  if (page==2) {
    serialLCD.write(254); 
    serialLCD.write(128);   
    serialLCD.write("                "); // clear display
    serialLCD.write("                ");
    if ( mode!=GSM_ONLY) { 
      serialLCD.write(254); 
      serialLCD.write(128);
      serialLCD.write("La:");
      serialLCD.print(latGPS,6);
      serialLCD.write(254); 
      serialLCD.write(192);
      serialLCD.write("Lo:");
      serialLCD.print(lonGPS,6); 
       
    }
    delay(2000);
  }
   if (page==1 && mode!=GSM_ONLY) {
    page+=1;}
  else page=1;
}

void sendDataServer(boolean mode, const String &IMEI, const String &data){
  
  char str_lat[9];
  char str_lon[9];
  char gps_mode[]="GSM";
  char data2[80];
  char url2[]="http://pi1.lab.hummelgard.com:88/addData";
  uint16_t statuscode;
  int16_t length;
 /*
  if (mode==GSM_ONLY){
    dtostrf(latGSM, 8, 5, str_lat);
    dtostrf(lonGSM, 8, 5, str_lon);
    sprintf(gps_mode,"GSM");
  }
  
  else {
    dtostrf(latGPS, 8, 5, str_lat);
    dtostrf(lonGPS, 8, 5, str_lon);
    sprintf(gps_mode,"GPS");
  }
  

       
  char buffer[23];      
  fona.getTime(buffer, 23);  
  char date[21];
  String(buffer).substring(1,22).toCharArray(date,21);
  sprintf(data2,"latitude=%s&longitude=%s&time=%s&mode=%s",str_lat,str_lon,date,gps_mode);

  fona.HTTP_POST_start(url2, F("application/x-www-form-urlencoded"), (uint8_t *) data2, strlen(data2), &statuscode, (uint16_t *)&length);
*/
}

void loop() {

//float husLongDegrees=17.178110;
//float husLatDegrees =62.165641;
//float distance = 0;

//int R = 6371;

//float   lat1 = husLatDegrees;
//float   lon1 = husLongDegrees;
int       page = 1;
float     latGPS;
float     latGSM;
float     lonGPS;
float     lonGSM;

String    IMEI_id;
boolean   mode;
float     dlat;
float     dlon;
float     a;
float     c;
uint16_t  batteryLevel;

  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated){
      watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;
  
       serialLCD.begin(9600);
   delay(500);
  pinMode(13, OUTPUT); 
  serialLCD.write(254); 
  serialLCD.write(128);   
  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  serialLCD.write(254); 
  serialLCD.write(128); 
  serialLCD.write("booting.");
  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == LOW;
  delay(2000);
  pinMode(FONA_POWER_KEY, INPUT);
  //delay(500);
  
  while (! Serial);

  Serial.begin(115200);
  Serial.println(F("Adafruit FONA 808 GPS demo"));
  Serial.println(F("Initializing FONA... (May take a few seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));
  serialLCD.write(".");
  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);
  delay(5000);
  serialLCD.write(".");
  fona.setGPRSNetworkSettings(F("online.telia.se"));
  fona.enableGPRS(true);
  
  if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
     Serial.println(F("Failed to enable"));
  serialLCD.write(".");  

     
      read_FONA_GPS(latGPS, lonGPS, latGSM, lonGSM, mode, IMEI_id, batteryLevel);
      printLCD(latGPS, lonGPS, latGSM, lonGSM, mode, page, batteryLevel);

    }
  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == LOW;
  delay(2000);
  pinMode(FONA_POWER_KEY, INPUT);
  }
  
  // Go to sleep!
  sleep();
  



 

}
