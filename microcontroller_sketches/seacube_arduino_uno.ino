/*
  File for SeaCube's onboard Arduino Uno implmeneting the following functionality:
  ---Polls PN532 RFID reader for tag detection (via I2C serial connection)
  ---Outputs a corresponding digital signal to the Teensy 4.1
      ---Logical 1 if tag detected
      ---Logical 0 if no tag detected
  ---Occassionally re-initialize I2C connection to reader in case of serial connection loss
*/


#include <Wire.h>
#include <Adafruit_PN532.h> //RFID library

/* Define RFID pin macros and variables  */
#define PN532_IRQ   2   // IRQ (interrupt request)
#define PN532_RESET 3   // Hardware reset
#define DIGITAL_OUT 8
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); //RFID object. Opens I2C address of RFID reader (0x48)
uint16_t timeout = 50; //Timeout to exit blocking card read

/* Initialize timing variables  */
unsigned long reconnectTimer = 0;
unsigned long currTime = 0;



void setup() {
  Serial.begin(115200);   //Serial port for debugging via serial monitor
  while (!Serial) {       //Waits for USB serial port to open
    delay(10);
  }

  pinMode(DIGITAL_OUT, OUTPUT);
  
  nfc.begin();            //Initialize I2C serial connection to reader
  uint32_t versiondata = nfc.getFirmwareVersion();
  while(!versiondata) {   //Could not find PN532 chip, try again
    delay(100);
    versiondata = nfc.getFirmwareVersion();
  }

  digitalWrite(DIGITAL_OUT, LOW);
  reconnectTimer = millis() + 20000;  //20 seconds from now
}



void loop() {
  currTime = millis();                  //Record current time
  if(currTime >= reconnectTimer) {      //Time to re-initialize I2C connection
    nfc.begin();  
    reconnectTimer = millis() + 20000;  //20 seconds from now
  }
  
  /* Declare variables to store returned RFID tag properties */
  uint8_t tagFound;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  //Stores returned UID (unique identifier) of RFID tag
  uint8_t uidLength;
  
  /* Attempt to populate variables with RFID tag data */
  tagFound = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, timeout);
  if(tagFound) {
    Serial.println("Tag detected");
    digitalWrite(DIGITAL_OUT, HIGH);
  }
  else if(!tagFound) {  
    Serial.println("No tag detected");
    digitalWrite(DIGITAL_OUT, LOW);
  }
  else {}
}
