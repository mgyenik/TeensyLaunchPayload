#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <TinyGPS.h>
#include <SdFat.h> 

#include "serial_util.h"

TinyGPS gps;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

UnifiedSensorWriter accelWriter(&accel);
UnifiedSensorWriter gyroWriter(&gyro);
UnifiedSensorWriter magWriter(&mag);
AltitudeSensorWriter bmpWriter(&bmp);

SensorWriter *writers[] = {
	&accelWriter,
	&gyroWriter,
	&magWriter,
	&bmpWriter,
};

// This flag enables debug sensor data dumps on the default Serial stream.
const bool debug = true;

// Chip Selector for SD connected to pin 10 on the Teensy 3.x
const int SDChipSelect = 4;
SdFat SD;

//Disable chip select since running multiple SPI devices
const int8_t DISABLE_CHIP_SELECT = -1;

// Print out the sensor data from the 10 DOF to USB for debug.
void displaySensorDetails(void) {
  sensor_t sensor;

  accel.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  gyro.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  mag.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  bmp.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  
  delay(500);
}

void transmitAndWriteSensorData() {
  File dataFile = SD.open("launch.txt", FILE_WRITE);
  for (int i = 0; i < ARRAY_SIZE(writers); i++) {
	// Sample each sensor and write the output to the SD card log file and the
	// XBee on Serial3. If debugging is enabled, sensors are also written to
	// Serial for viewing with the Serial Monitor.
	writers[i]->Update();
	writers[i]->Write(&dataFile);
	writers[i]->Write(&Serial3);
	if (debug) {
	  writers[i]->Write(&Serial);
	}
  }

  dataFile.close();
}


void transmitAndWriteGPSData() {
  float flat, flon;
  unsigned long age; 
  gps.f_get_position(&flat, &flon, &age);
  //transmit to Xbee and SD 
    
  Serial3.print("GPS:");
  Serial3.print("TIME:");
  Serial3.print(millis() / 1000.0);
  // Serial3.print(TinyGPS::_time);
  Serial3.print(",LAT:");
  Serial3.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial3.print(",LON:");
  Serial3.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  Serial3.println("");

  // write to SD card
  File dataFile = SD.open("launch.txt", FILE_WRITE);
      
  if(dataFile.isOpen()){
    dataFile.print("GPS:");
    dataFile.print("TIME:");
    dataFile.print(millis() / 1000.0);
    dataFile.print(",LAT:");
    dataFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    dataFile.print(",LON:");
    dataFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    dataFile.println("");
  }  
  dataFile.close();
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial3.begin(9600);
  while (!Serial);

  Serial.println(F("Let's launch some rockets!\n"));

  // Initialize the sensors.
  if(!accel.begin()){
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin()){
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Wire.beginTransmission(0x32); //address of LSM303
  Wire.write(0x23);  //address of the sensitivity control register LSM303
  Wire.write(0x30); //setting the range 0x00 -> +/-2g, 0x10 -> +/-4g, 0x20 -> +/- 8g, 0x30 -> +/-16g
  Wire.endTransmission();

  displaySensorDetails();
  Serial.print("\nInitializing SD card...");
  if (!SD.begin(SDChipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");
}

void loop(void){
  transmitAndWriteGPSData();
  for(int i =0; i < 5; i++) {
    transmitAndWriteSensorData();
    delay(200); /* measurements per second  1000 = 1 sec intervals*/
  }
}
