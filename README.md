# Introduction
Rocket application for sensate-x BIP in Darmstadt, Germany. The aim of the project is to design and implement multiple sensor technologies to be used for rocket flight data.
## Project goals
The aims for the project are:
* reach a minimum altitude of 300m
* integrate an altimeter
* measure peak flight velocity
* measure angular changes during flight
Data will be stored locally in internal EEPROM memory and will be retrieved using either a web application 
### Hardware
The rocket electronics are composed from:
* microcontroller (ESP32-S3)
* BME688 (temperature, pressure, humidity, gas sensor)
* QMI8658C Inertial Measurement Unit (accelerometer + gyroscope)
