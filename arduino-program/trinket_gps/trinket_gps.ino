/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */
#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 1
SoftwareSerial serialLCD(3,1); // pin 1 = TX, pin 3 = RX (unused)
SoftwareSerial serialGPS(0,2); // pin 1 = TX, pin 0= RX



// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  //pinMode(1, OUTPUT);
  serialLCD.begin(9600); // set up serial port for 9600 baud
  serialGPS.begin(9600); 
  delay(500); // wait for display to boot up
}

// the loop function runs over and over again forever
void loop() {
  serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(128);

  serialLCD.write("                "); // clear display
  serialLCD.write("                ");

  serialLCD.write(254); // move cursor to beginning of first line
  serialLCD.write(128);
 
  serialLCD.write("Hello, world!");
  
  //digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);              // wait for a second
  //digitalWrite(1, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}
