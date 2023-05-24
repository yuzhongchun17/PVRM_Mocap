
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050.h>
#include <TimerOne.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//
//# define MPU1_ADDRESS 0x69 //MPU1 with AD0 to HIGH
//# define MPU2_ADDRESS 0x68 //MPU2 with AD0 to LOW
 # define MPU1_ADDRESS 0x68 //MPU1 with AD0 to LOW
 # define MPU2_ADDRESS 0x69 //MPU2 with AD0 to HIGH
MPU6050 mpu1(MPU1_ADDRESS); // <-- use for AD0 high
MPU6050 mpu2(MPU2_ADDRESS);


// offset value from Calibration.ino
int offset1[] = {-3653, -427, 1590, -210, -10, -19}; // acelX acelY acelZ gyroX gyroY gyroZ for mpu1 0x68 
int offset2[] ={-5783, 861, 1207, 78, 18, -30}; //for mpu2 0x69 

// 6-deg raw data from imu1 & imu2
int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;
int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

// gyro & accel sensitivity
// reference: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
float gyroSensitivity = 131; // to achive [deg/s] for +-250 dps sensitivity , use raw_gyro_value/gryoSensitivity
float accelSensitivity = 16384; // to achive [m/s^2] for +-2 G sensitivity

String raw_acc_gyro_imu1          ; // String containing all raw data that is transmitted to PC 
String raw_acc_gyro_imu2; 

//Mocap Pin
const int en_mocap_Pin = 13;
int capture_time = 5000;


// time in millis
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long curMillis; // current ms
unsigned long prevMillis = 0;
unsigned long elapsedMillis; // time elapsed in ms
unsigned long captureDuration = 1000; //ms default recording period, could be changed later
String captureDurationInfo;

// #define LED_PIN 13
// bool blinkState = false;

void setup() {
  
    // join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    // Set the desired clock frequency in Hz (4 kHz)
    Wire.setClock(4000); 

    //Setup for mocap
    pinMode(en_mocap_Pin, INPUT_PULLDOWN);
    delay(100);
  
    // initialize mpu1
    Wire.beginTransmission(MPU1_ADDRESS);    
    Serial.println("Initializing MPU1...");
    mpu1.initialize();
    // verify connection
    Serial.println(mpu1.testConnection() ? "MPU1 connection successful" : "MPU1 connection failed");
    Wire.endTransmission(MPU1_ADDRESS);

    // initialize mpu2
    Wire.beginTransmission(MPU2_ADDRESS);    
    Serial.println("Initializing MPU2...");
    mpu2.initialize();

    // verify connection
    Serial.println(mpu2.testConnection() ? "MPU2 connection successful" : "MPU2 connection failed");
    Wire.endTransmission(MPU2_ADDRESS);


    // set offset value 
    mpu1.setXAccelOffset(offset1[0]);
    mpu1.setYAccelOffset(offset1[1]);
    mpu1.setZAccelOffset(offset1[2]);
    mpu1.setXGyroOffset(offset1[3]);
    mpu1.setYGyroOffset(offset1[4]);
    mpu1.setZGyroOffset(offset1[5]);

    mpu2.setXAccelOffset(offset2[0]);
    mpu2.setYAccelOffset(offset2[1]);
    mpu2.setZAccelOffset(offset2[2]);
    mpu2.setXGyroOffset(offset2[3]);
    mpu2.setYGyroOffset(offset2[4]);
    mpu2.setZGyroOffset(offset2[5]);

    //
//    // use the code below to change accel/gyro offset values
//    /*
//    Wire.beginTransmission(MPU1_ADDRESS);
//    Serial.println("Updating internal sensor offsets...");
//    Serial.print(mpu1.getXAccelOffset()); Serial.print("\t");  
//    Serial.print(mpu1.getYAccelOffset()); Serial.print("\t");  
//    Serial.print(mpu1.getZAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getXGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getYGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getZGyroOffset()); Serial.print("\t"); 
//    Serial.print("\n");
//    mpu1.setXGyroOffset(220);
//    mpu1.setYGyroOffset(76);
//    mpu1.setZGyroOffset(-85);
//    Serial.print(mpu1.getXAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getYAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getZAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getXGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getYGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu1.getZGyroOffset()); Serial.print("\t"); 
//    Serial.print("\n");
//    Wire.endTransmission(MPU1_ADDRESS);
//
//    Wire.beginTransmission(MPU2_ADDRESS);
//    Serial.println("Updating internal sensor offsets...");
//    Serial.print(mpu2.getXAccelOffset()); Serial.print("\t");  
//    Serial.print(mpu2.getYAccelOffset()); Serial.print("\t");  
//    Serial.print(mpu2.getZAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getXGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getYGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getZGyroOffset()); Serial.print("\t"); 
//    Serial.print("\n");
//    mpu2.setXGyroOffset(220);
//    mpu2.setYGyroOffset(76);
//    mpu2.setZGyroOffset(-85);
//    Serial.print(mpu2.getXAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getYAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getZAccelOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getXGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getYGyroOffset()); Serial.print("\t"); 
//    Serial.print(mpu2.getZGyroOffset()); Serial.print("\t"); 
//    Serial.print("\n");
//    Wire.endTransmission(MPU2_ADDRESS);
//    */
//
//    // uncomment these lines for calibration
//     Serial.print("press any key to start recording \n");
//     while(!Serial.available()) { }

}

// unsigned long previousTime = 0;
void loop() {
    //    Use serial monitor as command center to control the IMUs. For data logging, use Coolterm.exe. 
    Serial.println();
    Serial.println(F("type:"));
    Serial.println(F("c - calibration"));       // Before test trial, run this code. Make sure the 2 IMUs axes are parallel during calibration.                                         
    Serial.println(F("r - record data")); 

    // GIVEN INPUT FOR REACTION
    while (!Serial.available()) {} // waiting for input to arrive
    char serialInput = toLowerCase(Serial.read());
    
    // Discard extra Serial data.
    do {
      delay(10);
    } while (Serial.readString() == 'c' || Serial.readString() == 'r');

    switch(serialInput) {
      default:
      Serial.println();
      Serial.println(F("Invalid entry!!!"));
      break;

      // Calibrate IMU before collecting data
      case 'c':
      delay(10);
      // calibrateIMU();

      // record IMU data
      case 'r':
      delay(10);

      Serial.println("===== Enter capture duration (s) =====");
      
      // CAPTURE TIME SELECTION ---
      Serial.println(F("type:"));
      while(!Serial.available()){} // wait for input to arrive
      String inputDuration = Serial.readString();
      captureDuration = inputDuration.toInt() * 1000; // ms
      captureDurationInfo = String(captureDuration / 1000) + " s"; //s
      Serial.print("Capture duration set to " + captureDurationInfo);  
      Serial.println();
      // CAPTURE TIME SELECTION END ---

    //   //  WAIT FOR MOCAP SIGNAL ---
    //  Serial.println("");
    //  Serial.println("===== Wait for Mocap Signal to Start =====");
    //  while(!digitalRead(en_mocap_Pin)){}
    //  // WAIT FOR MOCAP END ---
      
      // READ IMU DATA ---
      startMillis = millis();
      while(1)
      {
        curMillis = millis();
        elapsedMillis = curMillis - startMillis;
        if(elapsedMillis >= captureDuration) // record IMU data for given 'captureDuration'
        {
          break;
        } else {
          // set data collecting frequency = 100 Hz (per 10 ms)
          // check if the desired time interval (10 ms) has passed
          if (curMillis - prevMillis >= 10) {
            prevMillis = curMillis;
            readIMU();
          }
        }
      }
      // READ IMU DATA END ---
    }
}

void readIMU() {
    // read raw accel/gyro measurements from MPU1 (6DOF)
    Wire.beginTransmission(MPU1_ADDRESS);
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    Wire.endTransmission(MPU1_ADDRESS);

    // read raw accel/gyro measurements from MPU2 (6DOF)
    Wire.beginTransmission(MPU2_ADDRESS);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    Wire.endTransmission(MPU2_ADDRESS);

    // print output datastring to serial monitor
    raw_acc_gyro_imu1 = String(ax1/accelSensitivity) + "/" + String(ay1/accelSensitivity) + "/" + String(az1/accelSensitivity) + "/" + String(gx1/gyroSensitivity) + "/" + String(gy1/gyroSensitivity) + "/" + String(gz1/gyroSensitivity); 
    raw_acc_gyro_imu2 = String(ax2/accelSensitivity) + "/" + String(ay2/accelSensitivity) + "/" + String(az2/accelSensitivity) + "/" + String(gx2/gyroSensitivity) + "/" + String(gy2/gyroSensitivity) + "/" + String(gz2/gyroSensitivity); 
    Serial.println(String(elapsedMillis) + "(ms)" + "/" + raw_acc_gyro_imu1 + "/" +raw_acc_gyro_imu2) + "/";
}
