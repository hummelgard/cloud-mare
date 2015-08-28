// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
SoftwareSerial mySerial(5,9); // pin 2 = TX, pin 3 = RX (unused)

void setup()
{
  pinMode(9, OUTPUT);
  mySerial.begin(9600); // set up serial port for 9600 baud
  delay(1500); // wait for display to boot up
}

void loop()
{
  mySerial.write(254); // move cursor to beginning of first line
  mySerial.write(128);

  mySerial.write("        1       "); // clear display
  mySerial.write("       1        ");

  mySerial.write(254); // move cursor to beginning of first line
  mySerial.write(128);
 
  mySerial.write("Hello 12345");

  while(1); // wait forever
}
