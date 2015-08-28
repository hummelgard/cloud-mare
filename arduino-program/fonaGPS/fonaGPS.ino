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


// standard pins for the 808 shield
#define FONA_RX 8
#define FONA_TX 9
#define FONA_RST 2

float husLongDegrees=17.178110;
float husLatDegrees =62.165641;
float distance = 0;
int page=1;
int R = 6371;

float   lat1 = husLatDegrees;
float   lon1 = husLongDegrees;
float   latGPS;
float   latGSM;
float   lonGPS;
float   lonGSM;


float   dlat;
float   dlon;
float   a;
float   c;
uint16_t batt;

int8_t fix;

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
}

void loop() {

  boolean gps_success = fona.getGPS(&latGPS, &lonGPS, &fix);
  Serial.print("gps_status: ");
  Serial.println(fix);
  boolean gsmloc_success = fona.getGSMLoc(&lonGSM, &latGSM);


  dlat = (lat1 - latGPS) / 180 * 3.14159;
  dlon = (lon1 - lonGPS) / 180 * 3.14159;
  a =sin(dlat/2) * sin(dlat/2) + cos(lat1/180*3.14159) * cos(latGPS/180*3.14159) * sin(dlon/2) * sin(dlon/2); 
  c = 2* atan2(sqrt(a),sqrt(1-a));
  distance= R * c * 1000;
  fona.getBattPercent(&batt);

  if (page==1) {
  serialLCD.write(254); 
  serialLCD.write(128);   
  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  serialLCD.write(254); 
  serialLCD.write(128);
  if(gps_success) {
    serialLCD.write("L=");
    serialLCD.print(distance,1);
    serialLCD.write("m");
  }
  
  serialLCD.write(254); 
  serialLCD.write(128+13);
  if (gsmloc_success){
    serialLCD.write("G"); 
  }
  serialLCD.write(254); 
  serialLCD.write(128+14);
  serialLCD.print(batt); 
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
    if (gps_success) { 
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

  char str_lat[9];
  char str_lon[9];
  char gps_mode[]="GSM";
  char data2[80];
  char url2[]="http://pi1.lab.hummelgard.com:88/addData";
  uint16_t statuscode;
  int16_t length;

  if (gps_success){
    dtostrf(latGPS, 8, 5, str_lat);
    dtostrf(lonGPS, 8, 5, str_lon);
    sprintf(gps_mode,"GPS");
  }
  else {
    dtostrf(latGSM, 8, 5, str_lat);
    dtostrf(lonGSM, 8, 5, str_lon);
    sprintf(gps_mode,"GSM");
  }

          
  char buffer[23];      
  fona.getTime(buffer, 23);  
  char date[21];
  String(buffer).substring(1,22).toCharArray(date,21);
  sprintf(data2,"latitude=%s&longitude=%s&time=%s&mode=%s",str_lat,str_lon,date,gps_mode);

  fona.HTTP_POST_start(url2, F("application/x-www-form-urlencoded"), (uint8_t *) data2, strlen(data2), &statuscode, (uint16_t *)&length);


  if (page==1 && gps_success) {
    page+=1;}
  else page=1;

}
