/*
  * Programmers: Matas Noreika, Luke Fleet, Natasha Palusinski, Enrique Sancho
  * Date: Thu Jul 3 10:29:15 AM IST 2025
  * Purpose: Rocket application for sensate-x BIP

Hardware: 
  * ESP32-S3
  * QMI8658C IMU (gyroscope & accelerometer)
  * BME688 temperature, humidity and pressure sensor
  * 1.14 inch TFT display (SPI) (Only for debugging)
*/

#include "rocket_application.h"

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

  //assign pre-calibrated accelerometer + gyroscope figures
  accCal.x = -0.008762;
  accCal.y = -0.064585;
  accCal.z = -0.075341;
  gyroCal.x = -4.297988;
  gyroCal.y = 0.663027;
  gyroCal.z = -0.635234;

  //initialise all sensors
  initQMI();
  initBME();
  init_nvs();
  //set the pressure referance on start up
  referancePressure = bme.readPressure()/100;
}

void loop() {

  //time in ms since microcontroller turned on
  currentTime = millis();

  //check if the magnitude of acceleration reached greater than 4m/s^2
  if(sensorData.accMag >= 5){
    toggleSensorLogging = true;
  }
  
  //check if its time to read sensor data
  if(currentTime - lastSensorReadTime >= SENSORREADTIME){
    
    readAltitude();
    readIMU();

    sensorData.timestamp = currentTime;
    sensorData.altitude = currentAltitude;
    sensorData.accMag = ((sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z) -1) * 9.81) + 0.3;
    sensorData.velocity += sensorData.accMag * (SENSORREADTIME/1000);
    sensorData.roll = kalmanAngleRoll;
    sensorData.pitch = kalmanAnglePitch;
    sensorData.yaw = angleYaw;
    
    #ifdef DEBUG
      printDataTFT(sensorData);
    #endif

    if(toggleSensorLogging){
      //convert sensor struct to csv data line
      sensor_data_to_csv(&sensorData,csv_line,sizeof(csv_line));

      #ifdef DEBUG
        Serial.println(csv_line);
      #endif

      save_csv_to_nvs(csv_line);
      
    }

    lastSensorReadTime = currentTime;
  }

  //used for data recollection + controls (simple CLI interface)
  if(Serial.available()){
    char input = Serial.read();

    switch(input){
      case 'r':
      memset(csv_buffer, 0, sizeof(csv_buffer));
      read_csv_from_nvs(csv_buffer,sizeof(csv_buffer));
      Serial.println(csv_buffer);
      break;
      case 'c':
      if(!clear_csv_from_nvs()){
        Serial.println("Erased CSV data!");
      }
      save_csv_header();//add a new header
      Serial.println("Header added");
      break;
      case 's':
      toggleSensorLogging = false;
      Serial.println("Stopped reading sensor readings");
      break;
    }
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

  void printDataTFT(sensor_data_t &data){
    tft.fillScreen(ST77XX_BLACK);//clear the screen
    tft.setCursor(0,0);
    tft.print("Time: ");
    tft.print((data.timestamp/1000));
    tft.println(" secs");
    tft.print("Altitude: ");
    tft.print(data.altitude);
    tft.println(" m");
    tft.printf("acc mag: %.2f\n", sensorData.accMag);
    tft.printf("velocity:%.2f\n",sensorData.velocity);
    // tft.print("acc.x: ");
    // tft.print(sensorData.acc.x);
    // tft.println(" g's");
    // tft.print("acc.y: ");
    // tft.print(sensorData.acc.y);
    // tft.println(" g's");
    // tft.print("acc.z: ");
    // tft.print(sensorData.acc.z);
    tft.println(" g's");
    tft.print("gyro.x: ");
    tft.print(sensorData.roll);
    tft.println(" deg");
    tft.print("gyro.y: ");
    tft.print(sensorData.pitch);
    tft.println(" deg");
    tft.print("gyro.z: ");
    tft.print(sensorData.yaw);
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

//function that calculated roll + pitch angles using accelerometer
void calcAccAngles(){
  accAngles.x = atan(acc.y/sqrt(acc.x*acc.x+acc.z*acc.z))*1/(PI/180);
  accAngles.y = -atan(acc.x/sqrt(acc.y*acc.y+acc.z*acc.z))*1/(PI/180);
}

//function to initialise the nvs of esp32
esp_err_t init_nvs(){
  esp_err_t err = nvs_flash_init();
  if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
    ESP_ERROR_CHECK(nvs_flash_erase());//we erase contents if memory is used
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  Serial.println("NVS initialised successfully");
  return ESP_OK;
}

//function to save csv data to nvs memory
esp_err_t save_csv_to_nvs(const char *csv_data){
  nvs_handle_t nvs_handle;//handle for csv data location
  esp_err_t err;

  //open NVS in read & write mode
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if(err != ESP_OK){
    Serial.printf("Error opening NVS handle: %s", esp_err_to_name(err));
    return err;
  }

  //hold the information of the size of current csv data
  size_t existing_size = 0;
  err = nvs_get_blob(nvs_handle, CSV_KEY, NULL, &existing_size);

  char *combined_data = NULL;

  //if no errors occurred and there is data present
  if(err == ESP_OK && existing_size > 0){
    //apppend to existing data
    combined_data = (char*)malloc(existing_size + strlen(csv_data) + 2);// +2 for new line and null terminator
    if(combined_data == NULL){
      nvs_close(nvs_handle);
      return ESP_ERR_NO_MEM;
    }

    //read existing data
    err = nvs_get_blob(nvs_handle, CSV_KEY, combined_data, &existing_size);
    if(err != ESP_OK){
      free(combined_data);
      nvs_close(nvs_handle);
      return err;
    }

    strcat(combined_data, "\n");
    strcat(combined_data, csv_data);

  }else{
    
    //first time saving or error reading existing data
    combined_data = (char*)malloc(strlen(csv_data) + 1);
    if(combined_data == NULL){
      nvs_close(nvs_handle);
      return ESP_ERR_NO_MEM;  
    }

  }

  //save combined data
  err = nvs_set_blob(nvs_handle, CSV_KEY, combined_data, strlen(combined_data) + 1);
  if(err != ESP_OK){
    Serial.printf("Error saving CSV data: %s\n", esp_err_to_name(err));
    free(combined_data);
    nvs_close(nvs_handle);
    return err;
  }

  //commit combined data
  err = nvs_commit(nvs_handle);
  if(err != ESP_OK){
    Serial.printf("Error saving CSV data: %s\n", esp_err_to_name(err));
    free(combined_data);
    nvs_close(nvs_handle);
    return err;
  }

  Serial.printf("CSV data saved successfully\n");
  free(combined_data);
  nvs_close(nvs_handle);
  return ESP_OK;

} 

//function to read saved data
esp_err_t read_csv_from_nvs(char *csv_data, size_t max_size){
  nvs_handle_t nvs_handle;
  esp_err_t err;

  //open NVS
  err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
  if(err != ESP_OK){
    Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
    return err;
  }

  //read CSV data
  size_t required_size = max_size;
  err = nvs_get_blob(nvs_handle, CSV_KEY, csv_data, &required_size);
  if(err != ESP_OK){
    if(err == ESP_ERR_NVS_NOT_FOUND){
      Serial.printf("CSV data not found in NVS\n");
    }else{
      Serial.printf("Error reading CSV data: %s\n", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
    return err;
  }

  Serial.printf("CSV data read successfully, size %d bytes\n", required_size);
  nvs_close(nvs_handle);
  return ESP_OK;
}

//function to clear csv data from nvs
esp_err_t clear_csv_from_nvs(){
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if(err != ESP_OK){
    Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
    return err;
  }

  err = nvs_erase_key(nvs_handle, CSV_KEY);
  if(err != ESP_OK){
    Serial.printf("Error erasing CSV data: %s\n", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_commit(nvs_handle);
  if(err != ESP_OK){
    Serial.printf("Error commiting NVS: %s\n", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  Serial.printf("CSV data cleared successfully\n");
  nvs_close(nvs_handle);
  return ESP_OK;

}

//generates the text labels accociated with data inputs
esp_err_t save_csv_header(){
  char *header = "timestamp,altitude,accMag,vel,roll,pitch,yaw";
  return save_csv_to_nvs(header);
}

//function to convert the sensor_data to string
void sensor_data_to_csv(sensor_data_t *data, char *csv_string, size_t max_size){
 snprintf(csv_string, max_size, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
 currentTime,data->altitude,data->accMag,data->velocity,data->roll,data->pitch,data->yaw); 
}