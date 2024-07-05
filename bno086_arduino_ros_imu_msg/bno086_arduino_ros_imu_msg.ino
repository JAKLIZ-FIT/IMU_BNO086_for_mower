

/*
  Using the BNO08x IMU

  This example shows how to output linear accelerometer values.

  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include <Wire.h>
#include <Adafruit_SleepyDog.h>

#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
#define BNO08X_INT  2
//#define BNO08X_INT  -1
#define BNO08X_RST  3
//#define BNO08X_RST  -1

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

float quatI;
float quatJ;
float quatK;
float quatReal;
float quatRadianAccuracy;

float lax;
float lay;
float laz;
byte linAccuracy;

float gx;
float gy;
float gz;

bool received_q = false;
bool received_a = false;
bool received_g = false;

int inactive_counter = 0;
int unsuccessful_connect_counter = 0;

unsigned long previousDebugMillis = 0;
#define DEBUG_INTERVAL_MILLISECONDS 50

unsigned long previousBlinkMillis = 0;
#define BLINK_INTERVAL_MILLISECONDS 500
bool ledPinIsHigh = false;

// basically the same as the code from the library
// for hard reset
void myResetIMU()
{
  //pinMode(BNO08X_RST, OUTPUT);
  //while(1){
  //  digitalWrite(BNO08X_RST, HIGH);
  //  delay(1000);
  //  digitalWrite(BNO08X_RST, LOW);
  //  delay(1000);
  //}

  //pinMode(BNO08X_RST, INPUT_PULLDOWN);
  //delay(1000);
  //pinMode(BNO08X_RST, INPUT_PULLUP);
  //delay(500);
  //digitalWrite(BNO08X_RST, LOW);
  //delay(200);
  //digitalWrite(BNO08X_RST, HIGH);
  //delay(200);
  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, HIGH);
  delay(10);
  digitalWrite(BNO08X_RST, LOW);
  delay(10);
  digitalWrite(BNO08X_RST, HIGH);
  delay(10);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  Serial.println("Wire begin after");

  myResetIMU();

  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  while (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    //while (1)
    //  ;
    delay(50);
    unsuccessful_connect_counter++;
    if (unsuccessful_connect_counter > 3){
      NVIC_SystemReset();
    }
    myResetIMU();
    Serial.println("tried to reset the IMU");
    delay(500); // TODO remove
  }
  Serial.println("BNO08x found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
  
  //Watchdog.enable(4000, true); // watchdog with a callback function 
  //Watchdog.enable(4000); // watchdog that will reset arduino on timeout
  Watchdog.enable(2000,false);

  WDT->INTENSET.bit.EW = 1;
  
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);
  NVIC_EnableIRQ(WDT_IRQn);
}

void watchdogCallback() {
//void handleWatchdogTimeout() {
  Serial.print("WatchDog Timeout, reseting IMU");
  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, HIGH);
  delay(10);
  digitalWrite(BNO08X_RST, LOW);
  delay(10);
  digitalWrite(BNO08X_RST, HIGH);
  delay(10);
  Watchdog.reset();
}

//void WDT_Handler() {
//  handleWatchdogTimeout();
//  //watchdogTimeout(); // Call the custom handler
//}
//void WDT_Handler() __attribute__((weak, alias("handleWatchdogTimeout"))); // callback for watchdog
  

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  
  if (myIMU.enableRotationVector(20) == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  //if (myIMU.enableLinearAccelerometer(20) == true) {
  //  Serial.println(F("Linear Accelerometer enabled"));
  //  Serial.println(F("Output in form x, y, z, in m/s^2"));
  //} else {
  //  Serial.println("Could not enable linear accelerometer");
  //}
  if (myIMU.enableGyro(20) == true) {
    Serial.println(F("Gyro enabled"));
    Serial.println(F("Output in form x, y, z, in radians per second"));
  } else {
    Serial.println("Could not enable gyro");
  }
}

void loop() {
  delay(10);

  Watchdog.reset();
  
  int timeSinceLastBlink = (millis() - previousBlinkMillis);
    
  if(timeSinceLastBlink > BLINK_INTERVAL_MILLISECONDS)
  {      
    previousBlinkMillis = millis();
    if (ledPinIsHigh){
      ledPinIsHigh = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    else{
      ledPinIsHigh = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {

    inactive_counter = 0;

    uint8_t reportID = myIMU.getSensorEventID();
    switch (reportID)
    {
    case SENSOR_REPORTID_ROTATION_VECTOR:
      quatI = myIMU.getQuatI();
      quatJ = myIMU.getQuatJ();
      quatK = myIMU.getQuatK();
      quatReal = myIMU.getQuatReal();
      quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      received_q = true;
      /*
      Serial.print(F("q,"));
      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy, 2);
      Serial.println();
      */
      break;
    case SENSOR_REPORTID_LINEAR_ACCELERATION:

      lax = myIMU.getLinAccelX();
      lay = myIMU.getLinAccelY();
      laz = myIMU.getLinAccelZ();
      linAccuracy = myIMU.getLinAccelAccuracy();

      received_a = true;
      /*
      Serial.print(F("l,"));
      Serial.print(lax, 2);
      Serial.print(F(","));
      Serial.print(lay, 2);
      Serial.print(F(","));
      Serial.print(laz, 2);
      Serial.print(F(","));
      Serial.print(linAccuracy);

      Serial.println();
      */
      
      
      break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:

      gx = myIMU.getGyroX();
      gy = myIMU.getGyroY();
      gz = myIMU.getGyroZ();

      received_g = true;
      /*
      Serial.print(F("g,"));
      Serial.print(gx, 2);
      Serial.print(F(","));
      Serial.print(gy, 2);
      Serial.print(F(","));
      Serial.print(gz, 2);
      Serial.println();
      */
      break;
    default:
      break;
    }
    int timeSinceLastSerialPrint = (millis() - previousDebugMillis);
    
    if(timeSinceLastSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
    {      
      previousDebugMillis = millis();
      
      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy, 2);
    
      Serial.print(",0.00,0.00,0.00,0");//Serial.print(F(",")); 
      //Serial.print(lax, 2);
      //Serial.print(F(","));
      //Serial.print(lay, 2);
      //Serial.print(F(","));
      //Serial.print(laz, 2);
      //Serial.print(F(","));
      //Serial.print(linAccuracy);
    
      Serial.print(F(","));
      Serial.print(gx, 2);
      Serial.print(F(","));
      Serial.print(gy, 2);
      Serial.print(F(","));
      Serial.print(gz, 2);
      
      //Serial.println();
      Serial.print(F(","));

      Serial.print(received_q);
      Serial.print(received_a);
      Serial.print(received_g);
      Serial.print(F(","));
      Serial.print(timeSinceLastSerialPrint);
      
      Serial.println();
      
      received_q = false;
      received_a = false;
      received_g = false;
    }
  }
  else{
    inactive_counter++;
    if (inactive_counter > 300){
      Serial.println("ARD: No IMU events in last 3s"); 
      myIMU.softReset(); 
      inactive_counter = 0;
    }
  }
}
