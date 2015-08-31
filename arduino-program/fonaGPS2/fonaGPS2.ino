/**
 *
 *
 */
#include "Adafruit_FONA_custom.h"
#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "crc16.h"

float data[500];
int       page = 1;


char    IMEI_id[15] = {0};

uint16_t  batteryLevel;

#define GSM_ONLY true

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   18       // Seconds to wait before a new sensor reading is logged.
                                                   
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

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);



int sleepIterations = 0;
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


void messageLCD(const int time, const String& line1, const String& line2=""){
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
  if (time > 0) 
    delay(time);
  else if (time < 0){
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


void initFONA808(){     
  fonaSerial->begin(4800);
  while (! fona.begin(*fonaSerial)) {
    pinMode(FONA_POWER_KEY, OUTPUT);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(100);   
    digitalWrite(FONA_POWER_KEY, LOW);
    delay(2000);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(100);
  }

  fona.enableGPS(true);
  delay(1000);
  //String* apn ="online.telia.se";
  //const char apn[] PROGMEM = "online.telia.se";
/*
  char buff[2];
  messageLCD(2000,"AT+CGATT=1");
  fona.sendCheckReply("AT+CGATT=1", "OK");

  messageLCD(2000,"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  fona.sendCheckReply("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK");

  messageLCD(2000,"AT+SAPBR=3,1,\"APN\",\"online.telia.se");
  fona.sendCheckReply("AT+SAPBR=3,1,\"APN\",\"online.telia.se", "OK");

  messageLCD(2000,"AT+SAPBR=3,1,\"USER\",\"0\"");
  fona.sendCheckReply("AT+SAPBR=3,1,\"USER\",\"0\"", "OK");

  messageLCD(2000,"AT+SAPBR=3,1,\"PWD\",\"0\"");
  fona.sendCheckReply("AT+SAPBR=3,1,\"PWD\",\"\"", "OK");
  
  messageLCD(2000,"AT+SAPBR=1,1"); 
  fona.sendCheckReply("AT+SAPBR=1,1", "OK");

fona.setGPRSNetworkSettings(F("online.telia.se"));
 fona.enableGPRS(true);
  float *lonGSM = 0;
  float *latGSM = 0;
  boolean check =0;
    while(check == 0){
      check = fona.getGSMLoc(lonGSM, latGSM);
  messageLCD(5000,"check="+String(check)); 
    }*/
    //messageLCD(5000,"check="+String(check)); 
  fona.setGPRSNetworkSettings(F("online.telia.se"));
  fona.enableGPRS(true);
  fona.enableNTPTimeSync(true, F("pool.ntp.org"));  
}

int8_t readFONA808(float *laGPS, float *loGPS,float *laGSM, float *loGSM, boolean *mode, char *IMEInr, uint16_t *batt){

  
  // Grab the IMEI number
  uint8_t imeiLen = fona.getIMEI(IMEInr);
  
  // Grab GPS latitude/longitude
  if (boolean gps_success = fona.getGPS(laGPS, loGPS)) {
    *mode = !GSM_ONLY;
  }
  else
    *mode = GSM_ONLY;

  //Grab GSM latitude/longitude data.
  boolean gsmloc_success = fona.getGSMLoc(loGSM, laGSM);
  fona.getBattPercent(batt); 

  return fona.GPSstatus();
}

void getGPSposFONA808(float *latAVG, float *lonAVG, float *fix_qualityAVG, int samples){
  int i = samples;
  short counterAVG = 0;  
  char      str_lat[15];
  char      str_lon[15];
  char      str_fix[3];
  float     latGPS = 0;
  float     latGSM = 0;
  float     lonGPS = 0;
  float     lonGSM = 0;
  int8_t    fix_quality = 0;
  boolean   mode;
  fona.getBattPercent(&batteryLevel);
  messageLCD(2000,"FONA col. data","Battery: " + String(batteryLevel)+ "%");
    
  do{    
    fix_quality = readFONA808(&latGPS, &lonGPS, &latGSM, &lonGSM, &mode, IMEI_id, &batteryLevel);
      
    if(fix_quality >= 2){
      *fix_qualityAVG += fix_quality;
      *latAVG += latGPS;
      *lonAVG += lonGPS;
      counterAVG++;  
      dtostrf(latGPS, 8, 5, str_lat);
      dtostrf(lonGPS, 8, 5, str_lon);
      messageLCD(0,String(str_lat) + " GPS " + String(batteryLevel),String(str_lon) + " #" + String(counterAVG));         
    }
  else {
     
      messageLCD(0,"Fix: " + String(fix_quality), String(i--) + "/" + String(samples) + "  batt% " + String(batteryLevel));  
    }
    delay(2000); 
       
  } while(i>0 && counterAVG <10 );
      
  *latAVG/=counterAVG;
  *lonAVG/=counterAVG;
  *fix_qualityAVG/=counterAVG;
  if (mode == GSM_ONLY){
    dtostrf(latGSM, 8, 5, str_lat);
    dtostrf(lonGSM, 8, 5, str_lon);
    messageLCD(3000,"Pos method:","GSM ONLY");
    messageLCD(8000,String(str_lat) + " GSM",String(str_lon));
  }
  else { 
    messageLCD(4000,"Pos method", "GPS");
    dtostrf(*latAVG, 8, 5, str_lat);
    dtostrf(*lonAVG, 8, 5, str_lon);
    dtostrf(*fix_qualityAVG, 4, 1, str_fix);
    messageLCD(8000,String(str_lat) + " GPS", String(str_lon) + ": "+ str_fix);
  }     
}

void closeFONA808(){
  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == HIGH;
  delay(500);   
  digitalWrite(FONA_POWER_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_POWER_KEY, HIGH);
  pinMode(FONA_POWER_KEY, OUTPUT);
  delay(500);
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
       
  char buffer[23];      
  fona.getTime(buffer, 23);  
  char date[21];
  String(buffer).substring(1,22).toCharArray(date,21);
  sprintf(data2,"latitude=%s&longitude=%s&time=%s&mode=%s",str_lat,str_lon,date,gps_mode);

  fona.HTTP_POST_start(url2, F("application/x-www-form-urlencoded"), (uint8_t *) data2, strlen(data2), &statuscode, (uint16_t *)&length);
*/
}
//SETUP
//-------------------------------------------------------------------------------------------
void setup() {
  serialLCD.begin(9600);
  delay(500);
  Serial.begin(115200);
  messageLCD(0,"booting.");
  /*
  Serial.print("hex=");
  unsigned short crc=0xFFFF; 
  //"123456789".getbytes(message,9)
  unsigned char* message =(unsigned char*) "123456789";
  Serial.println(crcsum(message, (unsigned long) 9, crc),HEX);
Serial.println(crcsum(message, (unsigned long) 9, crc));
 //crc16_update((uint16_t) 0xffff, (uint8_t)"1234567890");
  //Serial.println(crc16((unsigned char*)"1234567890", 10),HEX);
  */
  pinMode(FONA_POWER_KEY, OUTPUT);
  digitalWrite(FONA_POWER_KEY, HIGH);
  
  // This next section of code is timing critical, so interrupts are disabled.
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
}

//LOOP
//-------------------------------------------------------------------------------------------
void loop() {
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated){
      watchdogActivated = false;
      
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      
      messageLCD(0,"Awake!");// + (int) 8*sleepIterations);
      // Reset the number of sleep iterations.
      sleepIterations = 0;

      //DO SOME WORK!
      //Fire up FONA 808 GPS and take a position reading.
      messageLCD(0,"FONA power up");
      initFONA808();

      float latAVG = 0;
      float lonAVG = 0;
      float fix_qualityAVG = 0;
      getGPSposFONA808(&latAVG, &lonAVG, &fix_qualityAVG,10);

      messageLCD(2000, "FONA shutdown");
      closeFONA808();

      messageLCD(-2000, "Zzzz");


    }
  }
  // Go to sleep!
  sleep();
  
}
