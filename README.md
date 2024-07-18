# Pilot-Vitals-Monitoring-Jacket
Developed a flight jacket that monitors pilots' vitals including heart rate, external pressure and temperature, altitude, acceleration and gyroscopic values and smoke detection. Data is transmitted to the ThingSpeak cloud via ESP-NOW communication protocol between the transmitter and receiver.
This project utilizes ESP-32 Dev modules as microcontrollers at both the transmitting and receiving ends. 
It employs the MAX30102 sensor for monitoring heart rate and blood oxygen levels, the MPU6050 for acceleration and gyroscopic data, the BMP280 for external pressure and temperature, altitude measurements and lastly the MQ-2 gas sensor to detect smoke. Communication between the transmitter and receiver ESP-32 modules is established using the ESP-NOW wireless protocol operating in the 2.4 GHz band. 
The receiver leverages Wi-Fi to upload vital data to the ThingSpeak cloud, enabling data logging over time.

The required headers are: Wire.h, Adafruit_BMP280.h, esp-now.h, WiFi.h, MAX30105.h, heartRate.h, I2Cdev.h, MPU6050.h, ThingSpeak.h

The source code for both Transmitter and Receiver are provided.

The source code to obtain I2C address of all I2C device connected is also provided as "I2C_ADDR_FINDER.ino".

The source code to obtain MAC address of a ESP-32 module connected is also provided as "ESP_32_MAC_ADDR_FINDER.ino".
