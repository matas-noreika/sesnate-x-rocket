/*
  * Programmers: Matas Noreika, Luke Fleet, Natasha Palusinski, Enrique Sancho
  * Date: Thu Jul 3 10:29:15 AM IST 2025
  * Purpose: Rocket application API for sensate-x BIP

Hardware: 
  * ESP32-S3
  * QMI8658C IMU (gyroscope & accelerometer)
  * BME688 temperature, humidity and pressure sensor
  * 1.14 inch TFT display (SPI) (Only for debugging)
*/
#ifndef __ROCKET_APP_H__
#define __ROCKET_APP_H__ 1

//Macro to enable/disable serial prints & tft display
#define DEBUG 1

#include <Arduino.h>//core arduino library
#include <Wire.h>//I2C library
#include <SPI.h>//SPI library
#include <SensorQMI8658.hpp>//QMI8658C library
#include <Adafruit_BME680.h>//BME680-BME688 library
#include <stdlib.h>//libc utility library (malloc)
#include <string.h>//libc string library
#include <nvs_flash.h>//non-volatile storage flash (EEPROM) library

#ifdef DEBUG
  #include <Adafruit_ST7789.h>//TFT 1.14 inch library
#endif 

//Macro's for I2C
#define I2C_SDA 42
#define I2C_SCL 41

#ifdef DEBUG
  //Macro's for SPI
  #define SPI_SCK 36
  #define SPI_MISO 37
  #define SPI_MOSI 35  
  //Macro's defintions for TFT display
  #define TFT_DC 39
  #define TFT_CS 7
  #define TFT_RST 40
  #define TFT_BACKLIGHT 45
#endif

//Macro's for sensor Addresses
#define QMI_ADDR 0x6B
#define BME_ADDR 0x76

//Other macro's
#define SENSORREADTIME 1000.0//time in ms for sensor read interval
#define CALIBRATIONTIME 2.0//time (s) for single calibration to occur
#define CALIBRATIONSAMPLES 200//N samples to use for calibration
#define GYRO_DEVIATION 5//the deviation for individual gyro measurement in degrees
#define ACC_ANGLE_DEVIATION 2//the deviation for individual angle measurement from accelerometer in degrees
#define STORAGE_NAMESPACE "csv_storage" // namespaces are used to logically seperate different data storage units
#define CSV_KEY "csv_data"//the key/name of file to read from
#define MAX_CSV_SIZE 6450 // max nvs size is 4kB

typedef struct {
  int timestamp;
  float altitude;
  float accMag;
  float velocity;
  float roll;
  float pitch;
  float yaw; 
} sensor_data_t;

//global variables
float referancePressure;//variable used to set pressure referance for altitude
float currentAltitude;//variable used to store the result of altitude calculation from pressure
int currentTime;//current loop iteration time
float lastSensorReadTime = 0;//variable used to store the time in milli seconds of last sensor read
float kalman1DOutput[] = {0,0};//used to store the updated prediction and uncertainty
float kalmanAngleRoll = 0,kalmanAnglePitch = 0, angleYaw = 0;
float kalmanUncertaintyAnglePitch = 2*2,kalmanUncertaintyAngleRoll = 2*2;
char csv_buffer[MAX_CSV_SIZE];
char csv_line[256];
bool toggleSensorLogging = false;

//global objects
SensorQMI8658 qmi;
IMUdata gyroCal,accCal;//gyro and accelerometer calibration data
IMUdata gyro, acc;//gyro and acc data
IMUdata accAngles;//angles calculated using accelerometer (only pitch and roll)
Adafruit_BME680 bme(&Wire);
sensor_data_t sensorData;

#ifdef DEBUG
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);//object definition for tft display
#endif
//function prototypes
/********************************************
Function to initialise QMI8658C
********************************************/
bool initQMI(void);
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
/**********************************************
Function to perform kalman 1D operation on data
**********************************************/
void kalman_1d(float,float,float,float);
/**************************************************************
Function to use trigonometry to calculate pitch and roll angles
**************************************************************/
void calcAccAngles();
/**************************************************************
Function initialise the NVS
**************************************************************/
esp_err_t init_nvs();
/*************************************************************
Function to save data as csv to nvs
**************************************************************/
esp_err_t save_csv_to_nvs(const char *csv_data);
/**************************************************************
Function to read data as csv from nvs
**************************************************************/
esp_err_t read_csv_from_nvs(char *csv_data, size_t max_size);
/**************************************************************
Function to clear csv file from nvs
**************************************************************/
esp_err_t clear_csv_from_nvs();
/**************************************************************
Function to append csv header to file
**************************************************************/
esp_err_t save_csv_header();
/**************************************************************
Function to convert sensor data to string for csv
**************************************************************/
void sensor_data_to_csv(sensor_data_t *data, char *csv_string,size_t max_size);

#ifdef DEBUG
  /*********************************
  Function to initialise TFT display
  *********************************/
  bool initTFT(void);
  /********************************
  Function to print data out to TFT
  ********************************/
  void printDataTFT(sensor_data_t& data);
#endif

#endif