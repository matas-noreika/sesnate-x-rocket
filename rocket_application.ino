/*
  * Promgrammers: Matas Noreika, Luke Fleet, Natasha Palusinski, Enrique Sancho
  * Date: Thu Jul 3 10:29:15 AM IST 2025
  * Purpose: Rocket application for sensate-x BIP

Hardware: 
  * ESP32-S3
  * QMI8658C IMU (gyroscope & accelerometer)
  * BME688 temperature, humidity and pressure sensor
  * 1.14 inch TFT display (SPI) (Only for debugging)
*/

#include <Arduino.h>//core arduino library
#include <Wire.h>//I2C library
#include <SensorQMI8658.hpp>//QMI8658C library
#include <Adafruit_BME680.h>//BME680-BME688 library

//Macro's for I2C
#define I2C_SDA 42
#define I2C_SCL 41

//Macro's for sensor Addresses
#define QMI_ADDR 0x6B 
#define BME_ADDR 0x76

//Other macro's
#define SENSORREADTIME 1000//time in ms for sensor read interval

//global variables
float referancePressure;//variable used to set pressure referance for altitude
float currentAltitude;//variable used to store the result of altitude calculation from pressure
float currentTime;//current loop iteration time
float lastSensorReadTime = 0;

//global objects
SensorQMI8658 qmi;
IMUdata gyroCal,accCal;//gyro and accelerometer calibration data
IMUdata gyro, acc;//gyro and acc data
Adafruit_BME680 bme(&Wire);

//function prototypes

/********************************************
Function to initialise QMI8658C
********************************************/
bool initQMI(void);
/********************************************
Function to get gyro and acc calibration data
********************************************/
//bool calibrateQMI(&IMUdata gyro, &IMUdata acc);
/********************************************
Function to initialise BME688
********************************************/
bool initBME(void);
/**************************************************
Function to read altitude data into currentAltitude
**************************************************/
bool readAltitude(void);
/*****************************
Function to read gyro+acc data
*****************************/
bool readIMU(void);

void setup() {
  //initialise serial communication
  Serial.begin(115200);

  //initialise I2C bus
  if(!Wire.begin(I2C_SDA,I2C_SCL)){
    Serial.println("could not start I2C bus!");
    //enter endless loop
    while(1);
  }

  //initialise all sensors
  initQMI();
  initBME();
}

void loop() {

  //time in ms since microcontroller turned on
  currentTime = millis();

  //check if its time to read sensor data
  if(currentTime - lastSensorReadTime >= SENSORREADTIME){
    if(!readAltitude()){
      Serial.println("Error reading altitude!");
    }

    readIMU();
  }

}

//function definitions

bool initQMI(void){
  //initialise the QMI sensor
  if(!qmi.begin(Wire,QMI_ADDR,I2C_SDA,I2C_SCL)){
    Serial.println("could not connect to QMI6858C sensor!");
    return false;
  }

  //gyroscope configuration 
  qmi.configGyroscope(
    SensorQMI8658::GYR_RANGE_128DPS,//max rotation in degs is 128
    SensorQMI8658::GYR_ODR_896_8Hz,//896.8Hz
    SensorQMI8658::LPF_MODE_3 // 13.37% of ODR    
  );

  //configure the accelerometer
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_8G,
    SensorQMI8658::ACC_ODR_1000Hz,
    SensorQMI8658::LPF_OFF
  );

  //enable sensor to read gyro + acc
  qmi.enableGyroscope();
  qmi.enableAccelerometer();
  return true;
}

// bool calibrateQMI(&IMUdata gyro, &IMUdata acc){
//   return false;
//}

bool initBME(void){
  //initialise BME688 sensor
  if(!bme.begin(BME_ADDR)){
    Serial.println("Failed to initialise BME688!");
    return false;
  }
  //configure the operational settings for the bme
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_1);
  bme.setGasHeater(320,150); //320*C for 150ms (need to confirm with datasheet)
  //continously try to preform a read until successful
  while(!bme.performReading());
  referancePressure = bme.pressure/100.0;//save the pressure referance in hectopascals
  Serial.print("pressure referance: ");
  Serial.print(referancePressure);
  Serial.println(" hPa");
  return true;
}

bool readAltitude(void){
  //check if reading bme was successful
  if(!bme.performReading()){
    return false;
  }
  currentAltitude =  bme.readAltitude(referancePressure);
  Serial.printf("Altitude:%f m\n",currentAltitude);
  //Serial.printf("Temperature:%f *C\n",bme.readTemperature());
  return true;
}

bool readIMU(){
  
  //check if the IMU data is not ready to read
  if(!qmi.getDataReady){
    Serial.println("IMU data is not ready to read!");
    return false;
  }

  //check if accelerometer data is ready to read
  if(!qmi.getAccelerometer(acc.x,acc.y,acc.z)){
    Serial.println("Failed to retrieved accelerometer data!");
  }

  if(!qmi.getGyroscope(gyro.x,gyro.y,gyro.z)){
    Serial.println("Failed to retrieve gyroscope data!");
  }

  return true;
}