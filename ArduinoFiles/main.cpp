#include <Arduino.h>
#define N 8

void setup() {
  // start serial port at 9600 bps and wait for port to open:

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize the pin N
  pinMode(N,OUTPUT);

}

void loop() {

    uint8_t inByte;

  // if we get a valid byte, read analog ins:

  if (Serial.available() > 0) {

    // get incoming byte:
    inByte = Serial.read();

    if(inByte == 'H'){
      digitalWrite(N, HIGH);
    }else{
      digitalWrite(N, LOW);
    }


    Serial.flush(); // To flush the buffer.

    Serial.println(inByte);


  }

}
