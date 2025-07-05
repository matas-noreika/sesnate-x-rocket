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

//Macro to enable/disable serial prints & tft display
#define DEBUG 1

#include <Arduino.h>//core arduino library
#include <Wire.h>//I2C library
#include <SPI.h>//SPI library
#include <SensorQMI8658.hpp>//QMI8658C library
#include <Adafruit_BME680.h>//BME680-BME688 library

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

//struct definition of sensorData
struct SensorData {
  float timestamp;
  float altitude;
  IMUdata acc;
  IMUdata gyro;
};

//global variables
float referancePressure;//variable used to set pressure referance for altitude
float currentAltitude;//variable used to store the result of altitude calculation from pressure
float currentTime;//current loop iteration time
float lastSensorReadTime = 0;
float kalman1DOutput[] = {0,0};//used to store the updated prediction and uncertainty
float kalmanAngleRoll = 0,kalmanAnglePitch = 0, angleYaw = 0;
float kalmanUncertaintyAnglePitch = 2*2,kalmanUncertaintyAngleRoll = 2*2;

//global objects
SensorQMI8658 qmi;
IMUdata gyroCal,accCal;//gyro and accelerometer calibration data
IMUdata gyro, acc;//gyro and acc data
IMUdata accAngles;//angles calculated using accelerometer (only pitch and roll)
Adafruit_BME680 bme(&Wire);
SensorData sensorData;

#ifdef DEBUG
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);//object definition for tft display
#endif
//function prototypes
/********************************************
Function to initialise QMI8658C
********************************************/
bool initQMI(void);
/********************************************
Function to get gyro and acc calibration data
********************************************/
bool calibrateQMI();
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

#ifdef DEBUG
  /*********************************
  Function to initialise TFT display
  *********************************/
  bool initTFT(void);
  /********************************
  Function to print data out to TFT
  ********************************/
  void printDataTFT(SensorData& data);
#endif

void setup() {
  
  #ifdef DEBUG
    //initialise serial communication
    Serial.begin(115200);
    //initialise SPI bus
    SPI.begin(SPI_SCK,SPI_MISO,SPI_MOSI,TFT_CS);
    initTFT();
  #endif

  //initialise I2C bus
  if(!Wire.begin(I2C_SDA,I2C_SCL)){
    #ifdef DEBUG
      Serial.println("could not start I2C bus!");
    #endif
    //enter endless loop
    while(1);
  }

  //initialise all sensors
  initQMI();
  //delay(1);//add mini delay to prevent read fail
  calibrateQMI();
  initBME();
}

void loop() {

  //time in ms since microcontroller turned on
  currentTime = millis();

  //check if its time to read sensor data
  if(currentTime - lastSensorReadTime >= SENSORREADTIME){
    
    readAltitude();
    readIMU();

    sensorData.timestamp = currentTime;
    sensorData.altitude = currentAltitude;
    sensorData.acc = acc;
    sensorData.gyro.x = kalmanAngleRoll;
    sensorData.gyro.y = kalmanAnglePitch;
    sensorData.gyro.z = angleYaw;
    printDataTFT(sensorData);
    lastSensorReadTime = currentTime;
  }

}

//function definitions

bool initQMI(void){

  //initialise the QMI sensor
  if(!qmi.begin(Wire,QMI_ADDR,I2C_SDA,I2C_SCL)){
    #ifdef DEBUG
      Serial.println("could not connect to QMI6858C sensor!");
    #endif
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

bool calibrateQMI(){

  for(int i = 0; i < CALIBRATIONSAMPLES; i++){
    
    //wait until data is ready
    while(!qmi.getDataReady());
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyro.x, gyro.y, gyro.z);

    accCal.x += acc.x;
    accCal.y += acc.y;
    accCal.z += acc.z+1;
    gyroCal.x += gyro.x;
    gyroCal.y += gyro.y;
    gyroCal.z += gyro.z;

    delay(CALIBRATIONTIME/CALIBRATIONSAMPLES);
  }
  
  accCal.x /= CALIBRATIONSAMPLES;
  accCal.y /= CALIBRATIONSAMPLES;
  accCal.z /= CALIBRATIONSAMPLES;
  gyroCal.x /= CALIBRATIONSAMPLES;
  gyroCal.y /= CALIBRATIONSAMPLES;
  gyroCal.z /= CALIBRATIONSAMPLES;

  #ifdef DEBUG
    Serial.println("Calibration data: ");
    Serial.printf("acc.x:%f g's\n",accCal.x);
    Serial.printf("acc.y:%f g's\n",accCal.y);
    Serial.printf("acc.z:%f g's\n",accCal.z);
    Serial.printf("gyro.x:%f deg/s\n",gyroCal.x);
    Serial.printf("gyro.y:%f deg/s\n",gyroCal.y);
    Serial.printf("gyro.z:%f deg/s\n",gyroCal.z);
  #endif 

  return true;
}

bool initBME(void){
  
  //initialise BME688 sensor
  if(!bme.begin(BME_ADDR)){
    #ifdef DEBUG
      Serial.println("Failed to initialise BME688!");
    #endif
    return false;
  }
  //configure the operational settings for the bme
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_1);
  bme.setGasHeater(320,150); //320*C for 150ms (need to confirm with datasheet)
  
  while(!bme.performReading());
  referancePressure = bme.pressure/100;

  #ifdef DEBUG
    Serial.print("pressure referance: ");
    Serial.print(referancePressure);
    Serial.println(" hPa");
  #endif

  return true;
}

bool readAltitude(void){
  //check if reading bme was successful
  if(!bme.performReading()){
    return false;
  }
  currentAltitude =  bme.readAltitude(referancePressure);
  return true;
}

bool readIMU(){
  
  //check if the IMU data is not ready to read
  if(!qmi.getDataReady()){
    #ifdef DEBUG
      Serial.println("IMU data is not ready to read!");
    #endif
    return false;
  }

  //check if accelerometer data is ready to read
  if(!qmi.getAccelerometer(acc.x,acc.y,acc.z)){
    #ifdef DEBUG
      Serial.println("Failed to retrieved accelerometer data!");
    #endif
  }

  //check if the gyroscope data is ready to read
  if(!qmi.getGyroscope(gyro.x,gyro.y,gyro.z)){
    #ifdef DEBUG
      Serial.println("Failed to retrieve gyroscope data!");
    #endif
  }

  //apply calibration offsets
  acc.x -= accCal.x;
  acc.y -= accCal.y;
  acc.z -= accCal.z;
  gyro.x -= gyroCal.x;
  gyro.y -= gyroCal.y;
  gyro.z -= gyroCal.z;

  //determine the roll and pitch angles using accelerometer
  calcAccAngles();

  //perform kalman processing on roll angle
  kalman_1d(kalmanAngleRoll,kalmanUncertaintyAngleRoll,gyro.x,accAngles.x);
  kalmanAngleRoll = kalman1DOutput[0];
  kalmanUncertaintyAngleRoll = kalman1DOutput[1];
  //perform kalman processing on pitch angle
  kalman_1d(kalmanAnglePitch,kalmanUncertaintyAnglePitch,gyro.y,accAngles.y);
  kalmanAnglePitch = kalman1DOutput[0];
  kalmanUncertaintyAnglePitch = kalman1DOutput[1];
  //just integrate the yaw angle (unable to derive a measurement with accelerometer)
  angleYaw += gyro.z * (SENSORREADTIME/1000);
  return true;
}

#ifdef DEBUG
  bool initTFT(void){
    //backlight pin configurations
    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH);

    tft.init(135,240);//call init method to set screen size
    tft.setRotation(3);//can be value between 0-3
    tft.fillScreen(ST77XX_BLACK);//clear the screen
    //configure text display properties
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0,0);
    tft.print("TFT ready...");
    return true;
  }

  void printDataTFT(SensorData &data){
    tft.fillScreen(ST77XX_BLACK);//clear the screen
    tft.setCursor(0,0);
    tft.print("Time: ");
    tft.print((int)(data.timestamp/1000));
    tft.println(" secs");
    tft.print("Altitude: ");
    tft.print(data.altitude);
    tft.println(" m");
    tft.print("acc.x: ");
    tft.print(sensorData.acc.x);
    tft.println(" g's");
    tft.print("acc.y: ");
    tft.print(sensorData.acc.y);
    tft.println(" g's");
    tft.print("acc.z: ");
    tft.print(sensorData.acc.z);
    tft.println(" g's");
    tft.print("gyro.x: ");
    tft.print(sensorData.gyro.x);
    tft.println(" deg");
    tft.print("gyro.y: ");
    tft.print(sensorData.gyro.y);
    tft.println(" deg");
    tft.print("gyro.z: ");
    tft.print(sensorData.gyro.z);
    tft.println(" deg");
    /*tft.print("Velocity: ");
    tft.print(velocity);
    tft.println(" m/s");*/
  }
#endif

void kalman_1d(float kalmanState,float kalmanUncertainty,float kalmanInput,float kalmanMeasurement){
  kalmanState = kalmanState + (SENSORREADTIME/1000) * kalmanInput;// 1) perform prediction 
  kalmanUncertainty = kalmanUncertainty + (SENSORREADTIME/1000) * (SENSORREADTIME/1000) * GYRO_DEVIATION * GYRO_DEVIATION;// 2) determine uncertainty
  float kalmanGain = kalmanUncertainty * 1/(1*kalmanUncertainty+ACC_ANGLE_DEVIATION*ACC_ANGLE_DEVIATION);// 3) calculate kalman gain
  kalmanState = kalmanState + kalmanGain*(kalmanMeasurement-kalmanState);// 4) update the state using measurement
  kalmanUncertainty = (1-kalmanGain) * kalmanUncertainty;// 5) update the uncertainty
  
  //set the new update information as the kalman output data
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;
}

void calcAccAngles(){
  accAngles.x = atan(acc.y/sqrt(acc.x*acc.x+acc.z*acc.z))*1/(PI/180);
  accAngles.y = -atan(acc.x/sqrt(acc.y*acc.y+acc.z*acc.z))*1/(PI/180);
}