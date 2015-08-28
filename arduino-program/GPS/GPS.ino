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
int page=0;
int R = 6371;

float lat1 = husLatDegrees;
float   lat2;
float   lon1 = husLongDegrees;
float   lon2;
float   dlat;
float   dlon;
float   a;
float   c;
uint16_t batt;


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
  serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(128);
  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  pinMode(13, OUTPUT);  
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

  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);
  delay(5000);
  fona.setGPRSNetworkSettings(F("online.telia.se"));
  fona.enableGPRS(true);
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          Serial.println(F("Failed to enable"));
        
}

void loop() {

  float latitude, longitude, speed_kph, heading, speed_mph, altitude;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude);
  lat2 = latitude;
  lon2 = longitude;
  // print out the GSM location to compare
  //delay(2000 );
  boolean gsmloc_success = fona.getGSMLoc(&longitude, &latitude);

  serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(128);

  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  
  if (gps_success) {
    
    Serial.print("GPS lat:");
    Serial.println(latitude);
    Serial.print("GPS long:");
    Serial.println(longitude);
    Serial.print("GPS speed KPH:");
    Serial.println(speed_kph);
    Serial.print("GPS speed MPH:");
    speed_mph = speed_kph * 0.621371192;
    Serial.println(speed_mph);
    Serial.print("GPS heading:");
    Serial.println(heading);
    Serial.print("GPS altitude:");
    Serial.println(altitude);
    
    lat1 = husLatDegrees;
    lon1 = husLongDegrees;
    dlat = (lat1 - lat2) / 180 * 3.14159;
    dlon = (lon1 - lon2) / 180 * 3.14159;
    a =sin(dlat/2) * sin(dlat/2) + cos(lat1/180*3.14159) * cos(lat2/180*3.14159) * sin(dlon/2) * sin(dlon/2); 
    c = 2* atan2(sqrt(a),sqrt(1-a));
    distance= R * c * 1000;
  if (page==0) {


  serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(192);
  serialLCD.write("L=");
  serialLCD.print(distance,1);
  serialLCD.write("m");
  }
  if (page==1) {
    
  serialLCD.write(254); 
  serialLCD.write(128);
  //serialLCD.print("La:");
  serialLCD.write("S");
  serialLCD.print(lat2,4);
  serialLCD.write("/");
  serialLCD.print(lon2,4); 
  }
  if (page>1) {
    serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(128);

  serialLCD.write("                "); // clear display
  serialLCD.write("                ");
  serialLCD.write(254); 
  serialLCD.write(128);
  //serialLCD.print("La:");
  serialLCD.write("La:");
  serialLCD.print(lat2,6);
  serialLCD.write(254); 
  serialLCD.write(192);
  serialLCD.write("Lo:");
  serialLCD.print(lon2,6); 
  }
  //delay(500);
  
  if (distance < 100){
    digitalWrite(13, HIGH);
    }
  else{
  digitalWrite(13, LOW);
  }

   else {

    Serial.println("Waiting for FONA 808 GPS 3D fix...");
    //delay(500);
  }
      if (page==0) {
    serialLCD.write(254); // move cursor to beginning of first line
    serialLCD.write(128);
    fona.getBattPercent(&batt);
    serialLCD.write("battery "); 
    serialLCD.print(batt);
    serialLCD.write("%"); 
    //serialLCD.write(254); // move cursor to beginning of first line
    //serialLCD.write(192);
    //serialLCD.write("NO FIX          ");
    }


  if (gsmloc_success && gps_success) {
    if (page==0) {
    lat1 = latitude;
    lon1 = longitude;
    dlat = (lat1 - lat2) / 180 * 3.14159;
    dlon = (lon1 - lon2) / 180 * 3.14159;
    a =sin(dlat/2) * sin(dlat/2) + cos(lat1/180*3.14159) * cos(lat2/180*3.14159) * sin(dlon/2) * sin(dlon/2); 
    c = 2* atan2(sqrt(a),sqrt(1-a));
    distance= R * c * 1000;
    serialLCD.write(254); // move cursor to beginning of first line
    serialLCD.write(203);
    //serialLCD.write("dist: ");
    serialLCD.print(distance,0);
    serialLCD.write("m");   
      }
   }
   if (gsmloc_success) {
    if (page==1) {
    serialLCD.write(254); // move cursor to beginning of first line
    serialLCD.write(128);
    serialLCD.write("G");
    serialLCD.print(latitude,4);
    serialLCD.write("/");
    serialLCD.print(longitude,4);
    delay(1000);
    }
    Serial.print("GSM:");
    Serial.println(latitude);
    Serial.print("GSMLoc long:");
    Serial.println(longitude);
   }
   else {
    Serial.println("GSM location failed...");
  }

          char str_lat[9];
        char str_lon[9];
          char gps_mode[]="GSM";
       char data2[80];
 char url2[]="http://pi1.lab.hummelgard.com:88/addData";
         uint16_t statuscode;
        int16_t length;


            // Post data to website

        //char url[]="http://pi1.lab.hummelgard.com:88/login";
        //char data[]="username=admin&password=default";
        //Serial.println(data); 
     
        //fona.HTTP_POST_start(url, F("application/x-www-form-urlencoded"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length);
        





        if (gps_success){
          dtostrf(lat2, 8, 5, str_lat);
          dtostrf(lon2, 8, 5, str_lon);
          sprintf(gps_mode,"GPS");
        }
        else
          sprintf(gps_mode,"GSM");
        
        

         
          dtostrf(latitude, 8, 5, str_lat);
          dtostrf(longitude, 8, 5, str_lon);
          //sprintf(gps_mode,"GSM");
        
        char buffer[23];
        


        fona.getTime(buffer, 23);  
        char date[21];
        String(buffer).substring(1,22).toCharArray(date,21);
        
        Serial.println(date); 
        sprintf(data2,"latitude=%s&longitude=%s&time=%s&mode=%s",str_lat,str_lon,date,gps_mode);

        Serial.println(data2); 

        fona.HTTP_POST_start(url2, F("application/x-www-form-urlencoded"), (uint8_t *) data2, strlen(data2), &statuscode, (uint16_t *)&length);


    if (page<4) page+=1;
  if (gps_success) delay(10000);
  else delay(1000);
}
