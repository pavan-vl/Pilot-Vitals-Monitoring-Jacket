// Designed by Pavan VL
// Github: https://github.com/pavan-vl/Pilot-Vitals-Monitoring-Jacket
// LinkedIn: https://www.linkedin.com/in/pavan-v-l-2b1986266/

/* Variable List from Sensors
max30102 - beatsPerMinute(Float), beatsavg(Int)
mpu6050 - Xa,Ya,Za,Xg,Yg,Zg (Int)
bmp280 - bmp.readTemperature(),bmp.readPressure(),bmp.readAltitude(1013.25) (Float)
mq2 - analogRead(34) (Int)
*/

// Transmitter Code

#include <Wire.h>  // For i2c communication
#include <Adafruit_BMP280.h> // For BMP280 pressure sensor

#include <esp_now.h> // For establishing communication between two ESP-32
#include <WiFi.h> // For utilizing ESP-32 capabilities

// For MAX30102 heart beat sensor
#include <MAX30105.h> 
#include <heartRate.h>

// For MPU6050 accelerometer and gyroscope
#include <I2Cdev.h>     
#include <MPU6050.h>


// MAX30102
MAX30105 particleSensor; // Heart beat sensor object

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// BMP280
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS   10
#define dr 0x76 // Obtained I2C address of BMP280 module 

Adafruit_BMP280 bmp; // Pressure sensor object

// MPU6050
MPU6050 mpu;
int16_t ax, ay, az; // Acceleration raw values
int16_t gx, gy, gz; // Gyro raw values
int Xa=0,Ya=0,Za=0,Xg=0,Yg=0,Zg=0; // Mapped acceleration and gyro values

//ESP-NOW
uint8_t receiverAddress[] = {0x3C, 0x61, 0x05, 0x12, 0xA0, 0x5C}; // Obtained MAC address of the receiver
typedef struct struct_message { // Structure to send data
    int bpmavg;
    float beats;
    int Xacc;
    int Yacc;
    int Zacc;
    int Xgyro;
    int Ygyro;
    int Zgyro;
    float temp;
    float press;
    float alt;
    bool smoke;    
} struct_message;

struct_message txvalues; // Structure variable to send data

#define mqpin 34 // Connecting AO pin of MQ-2 to Pin34(ADC) of ESP-32

void setup() {
  
  Serial.begin(115200); 
  Serial.println("Initializing...");

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED

  // Initialize BMP280 
    
  while ( !Serial ) 
    delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); 
    Serial.println(bmp.sensorID(),16);    
    while (1) delay(10);
  }
  Serial.println(bmp.sensorID(),16);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering. 
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time. 

  
  Wire.begin(); // Initiate I2C
  mpu.initialize(); // Initialize MPU6050

  //ESP-NOW
   WiFi.mode(WIFI_STA);    
  if (esp_now_init() != ESP_OK) { // Initialize ESP-NOW
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    // Register receiver
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    // Add receiver
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    
}

void loop() {
  // Get IR value from heart beat sensor
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat; 
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of previous 4 readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  

  
  Serial.print("BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(",\tAvg BPM=");
  Serial.println(beatAvg);
  Serial.println();

  if (irValue < 50000){
    Serial.print(" No finger?");
    Serial.println();
  }


  //bmp280
   Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    Serial.println();
    

    //mpu6050
     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Xa = map(ax, -17000, 17000, -255, 255 );  // X axis data
  Ya = map(ay, -17000, 17000, -255, 255);   // Y axis data
  Za = map(az, -17000, 17000, -255, 255);   // Z axis data
  Xg = map(gx, -17000, 17000, -255, 255 );  // X axis data
  Yg = map(gy, -17000, 17000, -255, 255);   // Y axis data
  Zg = map(gz, -17000, 17000, -255, 255);   // Z axis data
  
  Serial.print("Axis X = ");
  Serial.print(Xa);
  Serial.print("  ");
  Serial.print("Axis Y = ");
  Serial.print(Ya);
  Serial.print("  ");
  Serial.print("Axis Z  = ");
  Serial.println(Za);
  Serial.print("Axis X = ");
  Serial.print(Xg);
  Serial.print("  ");
  Serial.print("Axis Y = ");
  Serial.print(Yg);
  Serial.print("  ");
  Serial.print("Axis Z  = ");
  Serial.println(Zg);
  Serial.println();

  Serial.print("Smoke Value: ");
  Serial.println(analogRead(mqpin));
  Serial.println();


  txvalues.beats= beatsPerMinute;
  txvalues.bpmavg= beatAvg;
  txvalues.Xacc= Xa;
  txvalues.Yacc= Ya;
  txvalues.Zacc= Za;
  txvalues.Xgyro= Xg;
  txvalues.Ygyro= Yg;
  txvalues.Zgyro= Zg;
  txvalues.temp= bmp.readTemperature();
  txvalues.press= bmp.readPressure();
  txvalues.alt= bmp.readAltitude(920); // 920m is the height of the area where the sensor is present above sea level
  /* Threshold set to 2000. If it the reading crosses 2000 it identifies it as smoke and sets the value to true
     If it the reading is below 2000 it identifies it as a safe scenario and sets the value to false*/
  txvalues.smoke=(analogRead(mqpin)>=2000)?true:false; 

  //Send data using ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &txvalues, sizeof(txvalues));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
        Serial.println();
    } else {
        Serial.println("Error sending the data");
        Serial.println();
    }
  
  delay(200);
}
