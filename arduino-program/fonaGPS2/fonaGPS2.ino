/**
 *
 *
 */
#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "Adafruit_FONA_custom.h"
#include "crc16.h"

#include <avr/pgmspace.h>
    // next line per http://postwarrior.com/arduino-ethershield-error-prog_char-does-not-name-a-type/
#define prog_char  char PROGMEM


// DEBUG levels, by hardware port 3 to set to high, level 3 can be set.
// Level 0=off, 1=some, 2=more, 3=most, 4=insane!

int DEBUG=0;
char readbuffer[60];

char data[10];
int dataIndex=0;
int dataCounter=0;

char    IMEI_id[15] = {0};

uint16_t  batteryLevel;

// Three debug levels, 0=off, 1=some, 2=everything


#define GSM_ONLY true

// Seconds to wait before a new sensor reading is logged.
#define LOGGING_FREQ_SECONDS   18       

// Number of times to sleep (for 8 seconds) 
#define MAX_SLEEP_ITERATIONS_GPS   LOGGING_FREQ_SECONDS / 8  

#define MAX_SLEEP_ITERATIONS_POST  MAX_SLEEP_ITERATIONS_GPS * 10 

                                                    

#define FONA_RX 8
#define FONA_TX 9
#define FONA_RST 2
#define FONA_POWER_KEY 5
#define FONA_PSTAT 4
#define DEBUG_PORT 3



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


/***SERIAL DISPLAY **********************************************************/

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



/***LOW LEVEL AT FONA COMMANDS***********************************************/

int ATreadFONA(int multiline, int timeout=10000){
Serial.print("timeout=");
  Serial.println(String(multiline));
  Serial.println(timeout);
  int replyidx=0;
  while (timeout--) {

    while(fonaSS.available()) {
      char c =  fonaSS.read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (replyidx == 0)   // the first 0x0A is ignored
          continue;

        if ( multiline == 0 ){
          timeout = 0;
          continue;
        }
        if ( multiline > 0 ){
          readbuffer[replyidx++] = ';';
          multiline--;
          continue; 
        }
      }
      readbuffer[replyidx] = c;
      if(DEBUG == 4){
        Serial.print("\t\t\t");Serial.print(c, HEX); Serial.print("#"); Serial.println(c);
      }
      else
        delay(20);
      
      replyidx++;
    }
    delay(1);
  }
  readbuffer[replyidx] = 0;  // null term
  if(DEBUG >= 3){
    messageLCD(20,"",readbuffer);
    Serial.print("\t\tREAD: ");
    Serial.println(readbuffer);
  }
  return replyidx;
  
}


/*
  int replyidx=0;
  while (timeout--) {
    while(fonaSS.available()) {
      char c =  fonaSS.read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (replyidx == 0)   // the first 0x0A is ignored
          continue;
        if ( !multiline) {
          
          }
      }  
      readbuffer[replyidx] = c;
      replyidx++;
    }
    if ( timeout == 0 ){
      break;
    }
      
    delay(1);
  }
  
  readbuffer[replyidx] = 0; 

  if(DEBUG >= 3){
    messageLCD(20,"",readbuffer);
    Serial.print("\t\tREAD: ");
    Serial.println(readbuffer);
  }
  
  return replyidx;
}
*/

int ATsendReadFONA(char* ATstring, int multiline=0, int timeout=10000){

  if(DEBUG >= 2){
    messageLCD(20, String(ATstring));
    Serial.print("\t\tSEND: ");
    Serial.println(String(ATstring));
  }
  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}

int ATsendReadFONA(const __FlashStringHelper *ATstring, int multiline=0, int timeout=10000){

  if(DEBUG >= 2){
    messageLCD(20, String(ATstring));
    Serial.print("\t\tSEND: ");
    Serial.println(String(ATstring));
  }
  fonaSS.println(String(ATstring));
  return ATreadFONA(multiline, timeout);
}

boolean ATsendReadVerifyFONA(char* ATstring, char* ATverify, int multiline=0, int timeout=10000){

  if(ATsendReadFONA(ATstring, timeout, multiline)){
    if( strcmp(readbuffer,ATverify) == 0 )
      return true;
    else
      return false;     
  }
}

boolean ATsendReadVerifyFONA(char* ATstring, const __FlashStringHelper *ATverify, int multiline=0, int timeout=10000){

  if(ATsendReadFONA(ATstring, timeout, multiline)){
    if( strcmp_P(readbuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;     
  }
}

boolean ATsendReadVerifyFONA(const __FlashStringHelper *ATstring, const __FlashStringHelper *ATverify, int multiline=0, int timeout=10000){
  if(ATsendReadFONA(ATstring, timeout, multiline)){
    if( strcmp_P(readbuffer, (prog_char*)ATverify) == 0 )
      return true;
    else
      return false;     
  }
}


/***GPRS COMMANDS************************************************************/

boolean enableGprsFONA(char* apn,char* user=0,char* pwd=0){

  if(DEBUG >= 2){
    messageLCD(500, "FONA gprs init",">OK");
    Serial.println("\tFONA gprs initializing.");
    return true;
  }  

 if( !ATsendReadVerifyFONA(F("AT+CIPSHUT"),F("SHUT OK")) )
  return false;
  
//"AT+CIPSHUT"), F("SHUT OK")

/*
 if(ATsendReadVerifyFONA(F("AT+CGATT?"),F("+CGATT: 0")) ){
    if(DEBUG >= 2){
      ATreadFONA(); //eat up OK
      messageLCD(500, "FONA gprs on","OK");
      Serial.println("FONA gprs is already on");
      return true;
    }  
  }
  else{
    if(DEBUG >= 2){
      ATreadFONA(); //eat up OK
      messageLCD(500, "FONA gprs off",">starting up");
      Serial.println("FONA gprs is off, >starting up");
    }  
  }
  
  delay(100);
  ATreadFONA(); //eat up OK
  */
  delay(100);
    
  if( !ATsendReadVerifyFONA(F("AT+CGATT=1"), F("OK")) )
    return false;
    
  if( !ATsendReadVerifyFONA(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), F("OK")) )
    return false;
    
  char AT_string[50] = "AT+SAPBR=3,1,\"APN\",\"";

  strcat(AT_string, apn);
  strcat(AT_string, "\"");
  if( !ATsendReadVerifyFONA(AT_string, "OK", 10000))
    return false;
    
  if(user){
    strcpy(AT_string,"AT+SAPBR=3,1,\"USER\",\"");
    strcat(AT_string, user);
    strcat(AT_string, "\"");
    if( !ATsendReadVerifyFONA(AT_string, "OK"))
      return false;
  }

  if(pwd){
    strcpy(AT_string,"AT+SAPBR=3,1,\"PWD\",\"");
    strcat(AT_string, pwd);
    strcat(AT_string, "\"");
    if( !ATsendReadVerifyFONA(AT_string, "OK"))
      return false;
  }
  
  if( !ATsendReadVerifyFONA(F("AT+SAPBR=1,1"), F("OK")) )
      return false;


  return true;  
}

boolean initFONA(){     
  
 fonaSS.begin(4800);
 
  // Check if FONA os ON, if not turn it on!
  if(digitalRead(FONA_PSTAT) == false ){
    if(DEBUG >= 2){
    messageLCD(1000, "FONA: off", ">power on");
    Serial.println("\tFONA is off, >power on");
    }
    pinMode(FONA_POWER_KEY, OUTPUT);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(100);
    digitalWrite(FONA_POWER_KEY, LOW);
    delay(2000);
    digitalWrite(FONA_POWER_KEY, HIGH);
    delay(7000);

  }

  // The following lines until func. end is required to
  //  make FONA work properley
  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);
  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);
  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);

  // turn off Echo!
  ATsendReadVerifyFONA(F("ATE0"), F("OK"));
  delay(100);
  if( !ATsendReadVerifyFONA(F("ATE0"), F("OK")) ){
    // FONA is on, do a reset!
    
    if(DEBUG >= 2){
      messageLCD(1000, "FONA: error", ">reseting");
      Serial.println("\tFONA error, >reseting");
    }
    pinMode(FONA_RST, OUTPUT);
    digitalWrite(FONA_RST, HIGH);
    delay(100);
    digitalWrite(FONA_RST, LOW);
    delay(100);
    digitalWrite(FONA_RST, HIGH);
    delay(7000);     
    }
 
  while (fonaSS.available()) fonaSS.read();

  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);
  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);
  ATsendReadVerifyFONA(F("AT"), F("OK"));
  delay(100);

  // turn off Echo! 
  ATsendReadVerifyFONA(F("ATE0"), F("OK"));
  delay(100);
  ATsendReadVerifyFONA(F("ATE0"), F("OK"));
  delay(100);

  // turn on hangupitude
  ATsendReadVerifyFONA(F("AT+CVHU=0"), F("OK"));
  delay(100);
 
  if( !enableGprsFONA("online.telia.se"))
    return false;


  
  //delay(100);
  
  //ATsendReadVerifyFONA(F("AT+CCLK?"),F("OK"));
  //delay(3000);
  
  /*
  //fona.setGPRSNetworkSettings(F("online.telia.se"));
  //fona.enableGPRS(true);
  //fona.enableNTPTimeSync(true, F("pool.ntp.org"));
  */
  return true;
}


boolean enableGpsFONA808(void){
  
  // first check if GPS is already on or off
  if (ATsendReadVerifyFONA(F("AT+CGPSPWR?"), F("+CGPSPWR: 1;OK"), 1) ){
    if(DEBUG >= 2){
      messageLCD(1000, "GPS power: on", ">OK");
      Serial.println("\tFONA GPS is already power on");
    }
    delay(100);
    return false;
  }
  else{
    if(DEBUG >= 2){
      messageLCD(1000, "GPS power: off", ">power on");
      Serial.println("\tFONA GPS power is off, turning it on");
    }    
    if (! ATsendReadVerifyFONA(F("AT+CGPSPWR=1"), F("OK")) )
      return false;
  }
  delay(100);
  return true;
 
  
}

int readGpsFONA808(char* latitude, char* longitude){

  int fix_status;
  if( ATsendReadVerifyFONA(F("AT+CGPSSTATUS?"), F("Location Not Fix;OK"), 1) )
    fix_status=1;
  else if(readbuffer[10] == '3')
    fix_status=3;
  else if(readbuffer[10] == '2')
    fix_status=2;
  else if(readbuffer[10] == 'U')
    fix_status=0;
  delay(100);

  if(fix_status >= 0){
    //ATsendReadVerifyFONA(F("AT+CGPSINF=2"),F("OK"),1);
    ATsendReadFONA(F("AT+CGPSINF=2"),1);
    
    delay(100);
    
    if(DEBUG >= 1){
    
      char displaybuff[32];
      for(int i=0;i<11;i++){
        displaybuff[i]=readbuffer[i+23];
        displaybuff[i+16]=readbuffer[i+35];
      }
      messageLCD(3000, displaybuff);
      //012345678901234567890123456789012345678901234567890
      //+CGPSINF: 2,090138.011,6209.9281,N,01710.5660,E,0,0,,128.7,M,881,N,0
      Serial.println();
      Serial.print("\tFONA GPSdata: ");
      Serial.print(fix_status);
      Serial.print(" GGA: ");
      Serial.println(readbuffer);
    }    
   
  }
  
  return fix_status;
}



int8_t readFONA808(float *laGPS, float *loGPS,float *laGSM, float *loGSM, boolean *mode, char *IMEInr, uint16_t *batt){

  
  // Grab the IMEI number
  int imeiLen = fona.getIMEI(IMEInr);
  
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



void getGPSposFONA808(char *latAVG_str, char *lonAVG_str, char *fix_qualityAVG_str, int samples){
  int i = samples;
  short counterAVG = 0;  

  char      str_fix[3];
  float     latGPS = 0;
  float     latGSM = 0;
  float     lonGPS = 0;
  float     lonGSM = 0;
  float     latAVG = 0;
  float     lonAVG = 0;
  float     fix_qualityAVG = 0;
    
  int8_t    fix_quality = 0;
  boolean   mode;
  fona.getBattPercent(&batteryLevel);
  messageLCD(2000,F("FONA col. data"),"Battery: " + String(batteryLevel)+ "%");
    
  do{    
    fix_quality = readFONA808(&latGPS, &lonGPS, &latGSM, &lonGSM, &mode, IMEI_id, &batteryLevel);
      
    if(fix_quality >= 2){
      fix_qualityAVG += fix_quality;
      latAVG += latGPS;
      lonAVG += lonGPS;
      counterAVG++;  
      dtostrf(latGPS, 9, 5, latAVG_str);
      dtostrf(lonGPS, 9, 5, lonAVG_str);
      messageLCD(0,String(latAVG_str) + " GPS " + String(batteryLevel),String(lonAVG_str) + " #" + String(counterAVG));         
    }
  else {
      fix_qualityAVG += fix_quality;
      latAVG += latGSM;
      lonAVG += lonGSM;
      counterAVG++;  
      dtostrf(latGSM, 9, 5, latAVG_str);
      dtostrf(lonGSM, 9, 5, lonAVG_str);
      messageLCD(0,String(latAVG_str) + " GSM " + String(batteryLevel),String(lonAVG_str) + " #" + String(counterAVG));         
    
      //messageLCD(0,"Fix: " + String(fix_quality), String(i--) + "/" + String(samples) + "  batt% " + String(batteryLevel));  
    }
    delay(2000); 
       
  } while(i>0 && counterAVG < samples );
      
  latAVG/=counterAVG;
  lonAVG/=counterAVG;
  Serial.print("latAVG=");
  Serial.println(String(latAVG));
  Serial.print("lonAVG=");
  Serial.println(String(lonAVG));
  fix_qualityAVG/=counterAVG;
  dtostrf(latAVG, 9, 5, latAVG_str);
  dtostrf(lonAVG, 9, 5, lonAVG_str);
  
    Serial.print("latAVG_str=");
  Serial.println(String(latAVG_str));
    Serial.print("lonAVG_str=");
  Serial.println(String(lonAVG_str));
  /*
  if (mode == GSM_ONLY){
    dtostrf(latGSM, 9, 5, str_lat);
    dtostrf(lonGSM, 9, 5, str_lon);
    messageLCD(3000,F("Pos method:"),F("GSM ONLY"));
    messageLCD(8000,String(str_lat) + " GSM",String(str_lon));
  }
  else { 
    messageLCD(3000,F("Pos method"), F("GPS"));
    //dtostrf(*latAVG, 9, 5, str_lat);
    //dtostrf(*lonAVG, 9, 5, str_lon);
    //dtostrf(*fix_qualityAVG, 4, 1, str_fix);
    messageLCD(8000,String(latAVG_str) + " GPS", String(lonAVG_str) + ": "+ fix_qualityAVG_str);
    
  } */    
}

void powerOffFONA(){
  pinMode(FONA_POWER_KEY, OUTPUT);
  FONA_POWER_KEY == HIGH;
  delay(500);   
  digitalWrite(FONA_POWER_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_POWER_KEY, HIGH);
  pinMode(FONA_POWER_KEY, OUTPUT);
  delay(500);
}


int sendDataServer(char* url, char *data){
  
  data[dataIndex++]='#'; 
  data[dataIndex++]='1'; 
  data[dataIndex++]='#'; 

  unsigned short crc;
  //crcsum((const unsigned char*)data,dataIndex,crc);
  char buf [6];
  sprintf (buf, "%06i", crc);; 
Serial.println(crc);
  
  uint16_t statuscode;
  int16_t length;

  //String(buffer).substring(1,22).toCharArray(date,21);
  //sprintf(data2,"latitude=%s&longitude=%s&time=%s&mode=%s",str_lat,str_lon,date,gps_mode);

  //fona.HTTP_POST_start(url, F("application/x-www-form-urlencoded"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length);
  
  
  Serial.print("The HTTP POST status was:");
  Serial.println(statuscode);
  Serial.println(data);
  return statuscode;
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
  pinMode(DEBUG_PORT, INPUT);

  // You can set max debug level by hardware port: DEBUG_PORT, put HIGH
  if(digitalRead(DEBUG_PORT) == true)
    DEBUG = 3;
  
  serialLCD.begin(9600);
  delay(500);
  Serial.begin(115200);

  pinMode(FONA_PSTAT, INPUT);
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
    if (sleepIterations >= MAX_SLEEP_ITERATIONS_GPS) {
      if(DEBUG >= 1){
        messageLCD(1000, "AWAKE!",">booting");
        Serial.println("Awake!, -booting.");
      }
      
      // Reset the number of sleep iterations.
      sleepIterations = 0;

      //DO SOME WORK!
      //Fire up FONA 808 GPS and take a position reading.
      if(DEBUG >= 2){
        messageLCD(0, "FONA:",">power up");
        Serial.println("\tFONA power up.");
      }
      initFONA();

      if(DEBUG >= 2){
        messageLCD(0, "FONA:",">GPS power on");
        Serial.println("\tFONA GPS power on.");
      }

      char latAVG_str[12] = "0";
      char lonAVG_str[12] = "0";
      char fix_qualityAVG_str[5] = "0";

      enableGpsFONA808();
      delay(5000);
      
      readGpsFONA808(latAVG_str,lonAVG_str);
      if(DEBUG >= 3){
        messageLCD(0, latAVG_str,lonAVG_str);
        Serial.print("Lat/lon:");
        Serial.print(latAVG_str);
        Serial.print(" / ");
        Serial.println(lonAVG_str);
      }     


      
      //getGPSposFONA808(latAVG_str, lonAVG_str, fix_qualityAVG_str,3);
/*

      Serial.println("");
      Serial.println(dataIndex);
      Serial.println("lat=");
      for(char i=0;i<9;i++){
        data[dataIndex++]=latAVG_str[i]; 
        Serial.print(latAVG_str[i]);
        }
      Serial.println("");
      Serial.println(dataIndex);
      data[dataIndex++]='#';

      Serial.println("lon=");
      for(char i=0;i<9;i++){
        data[dataIndex++]=lonAVG_str[i]; 
        Serial.print(lonAVG_str[i]);
      }    
      data[dataIndex++]='#';
      Serial.println("");
      Serial.println(dataIndex);
      char dateAndTime[23];
      fona.getTime(dateAndTime,23);

      data[dataIndex++]=dateAndTime[1];
      data[dataIndex++]=dateAndTime[2];
      data[dataIndex++]=dateAndTime[4];
      data[dataIndex++]=dateAndTime[5];
      data[dataIndex++]=dateAndTime[7];
      data[dataIndex++]=dateAndTime[8];
      data[dataIndex++]='#';    
      data[dataIndex++]=dateAndTime[10];
      data[dataIndex++]=dateAndTime[11];
      data[dataIndex++]=dateAndTime[13];
      data[dataIndex++]=dateAndTime[14];
      data[dataIndex++]=dateAndTime[16];
      data[dataIndex++]=dateAndTime[17];    
      data[dataIndex++]='#';                        
     // for(short i=1;i<9;i++)
      //  data[dataIndex++]=dateAndTime[i];
      //  data[dataIndex++]=  
      
     // data[dataIndex++]='#'; 
     // for(short i=10;i<18;i++)          
     //   data[dataIndex++]=dateAndTime[i]; 
        
      //data[dataIndex++]='#';     
      data[dataIndex]='1';
      dataIndex+=1;
      data[dataIndex++]='#';     
      data[dataIndex]='2';
      dataIndex+=1;
      dataCounter++;
      Serial.print("data0=");
      Serial.println(data);
      */

      if(DEBUG >= 2){
        messageLCD(0, "FONA: ",">power off");
        Serial.println("\tFONA shuting down.");
      }
      powerOffFONA();


      dataCounter++;
      // Go to sleep!
      if(DEBUG >= 1){
        Serial.println("Going to sleep, -Zzzz.");
        messageLCD(-1000, "Go to sleep",">Zzzz.");
      }
      

    }
    if(dataCounter % 5 ==0) {
      //if(true == sendDataServer("http://pi1.lab.hummelgard.com:88/addData", data));
      dataIndex=0;
    }
   
  }

  sleep();
  
}
