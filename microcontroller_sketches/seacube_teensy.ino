/*
  File for SeaCube's onboard Teensy 4.1 implmeneting the following functionality:
  ---Collects BNO085 IMU accelerometer output at 200 Hz
  ---Applies high-pass filtering algorithm to IMU output at 200 Hz
  ---Generates a formatted string compiling the following data at 25 Hz:
      ---Limit switch output (digital signal -- HIGH or LOW)
      ---RFID reader output (digital signal from Arduino Uno -- HIGH or LOW)
      ---INA169 current sensing breakout board output (analog signal)
      ---Raw IMU x, y, and z accelerometer output (via I2C serial connection)
      ---Largest high-pass filter output of previous 8 iterations of 200 Hz loop
  ---Outputs formatted string to to the RPi over USB serial at 25 Hz.
  
  The formatted string follows this format:
  [Switch],[RFID],[INA169],[XVec],[YVec],[ZVec],[HPFilter]
*/


#include <Adafruit_BNO08x.h>  //IMU library
#include <Wire.h>

/* Define pin macros */
#define RFID_PIN 40
#define SWITCH_PIN 39
#define CHARGING_PIN 38

/* Variables for serial output: */ 
bool serialSwitch = 0;  
bool serialRFID = 0;    
int serialCharging = 0; 
float serialXVec = 0.0;
float serialYVec = 0.0;
float serialZVec = 0.0;
float serialHPFilter = 0.0;
String outputString = "";

/* Define IMU macros and variables: */
#define SENSOR_RATE 5000  //period at which IMU generates new data in microseconds (5000 us period = 200Hz)
#define BNO08X_RESET -1   //No reset necessary for I2C
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

/* Initialize high-pass filter variables */
float imu_y = 0;
float imu_y_prev = 0;
float imu_x = 0;
float imu_x_prev = 0;
float imu_alpha = 0.9;
float largestFilterOutput = 0.0;  //Stores largest magnitude HP filter output
float temp = 0.0;                 //Temporary variable used for calculation

/* Initialize timing variables */
unsigned long startItrTime = 0;   //Timer which maintains fixed 200Hz sampling rate of IMU
int serialCounter = 0;            //Records when to output data to RPi (every 8 iterations of 200 Hz loop)



void setup() {
  Serial.begin(115200); //Port for serial output to RPi
  while (!Serial) {     //Waits for USB serial port to open
    delay(10);
  }

  /********************************SET UP IMU***********************************/
  Wire.begin();                     //Initialize I2C on default pins
  for(int i = 0; i < 10; i++) {     //Make several attempts to initialize I2C connection to IMU
    if(!bno08x.begin_I2C()) {       //Failed to find BNO08x chip
      delay(500);
    }
    else {
      break;
    }
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER, SENSOR_RATE)) {   //Enable IMU accelerometer output
    //Serial.println("Could not enable accelerometer");
  }


  /****************************SET UP LIMIT SWITCH******************************/
  pinMode(SWITCH_PIN, INPUT);

  /*******************************SET UP RFID***********************************/
  pinMode(RFID_PIN, INPUT);
  
  /******************************SET UP CHARGING*********************************/
  pinMode(CHARGING_PIN, INPUT);

  delay(3000);
}



/* Builds output string with this format: [Switch],[RFID],[INA169],[XVec],[YVec],[ZVec],[HPFilter] */
void buildOutputString() {
  outputString = outputString + String(serialSwitch) + "," + String(serialRFID) + "," + String(serialCharging) + ",";
  outputString = outputString + String(serialXVec) + "," + String(serialYVec) + "," + String(serialZVec) + ",";
  outputString = outputString + String(serialHPFilter);
}



/* Ptints formatted raw sensor data to serial monitor for debugging */
void debugPrint() { 
  Serial.print("Switch: ");
  Serial.println(serialSwitch);
  Serial.print("RFID: ");
  Serial.println(serialRFID);
  Serial.print("Charging: ");
  Serial.println(serialCharging);
  Serial.print("XVec: ");
  Serial.print(serialXVec);
  Serial.print(". YVec: ");
  Serial.print(serialYVec);
  Serial.print(". ZVec: ");
  Serial.println(serialZVec);
  Serial.print(". HPFilter: ");
  Serial.println(serialHPFilter);
  Serial.println();
}




void loop() {
  startItrTime = micros();  //Record start time of current iteration

  /******************************COLLECT IMU DATA******************************/
  if (bno08x.getSensorEvent(&sensorValue)) {            //New data available from IMU
    if(sensorValue.sensorId == SH2_ACCELEROMETER) {     //New accelerometer data available
      serialXVec = sensorValue.un.accelerometer.x;
      serialYVec = sensorValue.un.accelerometer.y;
      serialZVec = sensorValue.un.accelerometer.z;
    }
  }


  /****************************APPLY HIGH-PASS FILTER****************************/
  imu_x_prev = imu_x;
  imu_x = serialYVec;
  imu_y_prev = imu_y;
  imu_y = (imu_alpha * imu_y_prev) + imu_alpha * (imu_x - imu_x_prev);
  if(imu_y < 0) {         //Convert filter output to positive number
    temp = imu_y * -1;
  }
  else {
    temp = imu_y;
  }
  if(temp > largestFilterOutput) {  //Store largest filter output
    largestFilterOutput = temp;
  }


  serialCounter++;  //Records number of iterations since last output to RPi


  /**********************SELECTIVELY DELAY TO MAINTAIN 200HZ**********************/
  if(serialCounter != 8) {
    while(micros() - startItrTime < 5000) {   //Not outputting to RPi this iteration, delay normally 
      delayMicroseconds(1);
    } 
  }
  else {
    while(micros() - startItrTime < 4967) {   //Outputting to RPi, which takes about 37 microsec, delay by 37 microsec less
      delayMicroseconds(1);
    } 
  }
  


  /********************************OUTPUT TO RPI********************************/
  if(serialCounter == 8) {  //Time to output serial data to RPi (outputs to RPi at 25Hz)
    
    /* Collect limit switch output */
    if(digitalRead(SWITCH_PIN) == HIGH) {
      serialSwitch = 1;
    }
    else {
      serialSwitch = 0;
    }

    /* Collect RFID output */
    if(digitalRead(RFID_PIN) == HIGH) {
      serialRFID = 1;
    }
    else {
      serialRFID = 0;
    }
    
    /* Collect INA169 output */
    serialCharging = analogRead(CHARGING_PIN);

    /* Collect largest high-pass filter output */
    serialHPFilter = largestFilterOutput;

    buildOutputString();            //Update global 'outputString' variable    
    Serial.println(outputString);   //Output to RPi over USB serial
    outputString = "";              //Reset for next serial output to RPi
    serialCounter = 0;              //Reset  
    largestFilterOutput = 0.0;      //Reset
  }
}


