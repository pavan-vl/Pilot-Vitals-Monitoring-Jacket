// Designed by Pavan VL
// Github: https://github.com/pavan-vl/Pilot-Vitals-Monitoring-Jacket
//LinkedIn: https://www.linkedin.com/in/pavan-v-l-2b1986266/

// Receiver Code
#include <WiFi.h> // For utilizing ESP-32 capabilities
#include <ThingSpeak.h> // For accessing ThingSpeak Cloud

#include <esp_now.h> // For establishing communication between two ESP-32


int bpmavgrx, // To receive bmp average 
xa,ya,za,xg,yg,zg; // To receive acceleration and gyro values
float beatsrx, // To receive heart rate value
temperaturerx,pressurerx,altituderx; // To receive temperature, pressure and acceleration values respectively
bool mq; // To receive smoke presence value


// Structure to receive data
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

struct_message incomingData; // Structure variable to receive data

// Callback function that will be executed when data is received
void onDataReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
    memcpy(&incomingData, data, sizeof(incomingData));
    Serial.print("Data received: ");
    bpmavgrx = incomingData.bpmavg;
    beatsrx = incomingData.beats;
    xa = incomingData.Xacc;
    ya = incomingData.Yacc;
    za = incomingData.Zacc;
    xg = incomingData.Xgyro;
    yg = incomingData.Ygyro;
    zg = incomingData.Zgyro;
    temperaturerx = incomingData.temp;
    pressurerx = incomingData.press;
    altituderx = incomingData.alt;
    mq = incomingData.smoke;
}

const char* ssid = "YourWiFi";// your network SSID (name) 
const char* password = "YourPassword";// your network password

WiFiClient  client; // Object for ThingSpeak WiFi client 

unsigned long myChannelNumber = 1; // ThingSpeak Channel Number
const char *myWriteAPIKey = "PFNGF4QFZPOI2ZIV"; // To permit writing to given channel in ThingSpeak


void setup() {
  Serial.begin(115200);     //Initialize serial

  WiFi.mode(WIFI_STA);   // Set ESP-32 as WifI Station
  WiFi.begin(ssid,password); // Connect to given Access Point
  ThingSpeak.begin(client);  // Initialize ThingSpeak

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataReceive); // Receive data by calling the call-back function
}

void loop() {
    
    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(500);     
      } 
      Serial.println("\nConnected.");
      digitalWrite(2,HIGH); // Asserts BUILT_IN LED on ESP-32 turning it ON
    }

  Serial.println("BPM= "+ String(beatsrx));
  //Serial.println("Average BPM= "+ String(bpmavgrx));
  Serial.println("Temp= "+ String(temperaturerx));
  Serial.println("Pressure= "+ String(pressurerx));
  Serial.println("Altitude= "+ String(altituderx));
  Serial.print("Xa= "+ String(xa)+"\t");
  Serial.print("Ya= "+ String(ya)+"\t");
  Serial.print("Za= "+ String(za)+"\t");
  Serial.print("Xg= "+ String(xg)+"\t");
  Serial.print("Yg= "+ String(yg)+"\t");
  Serial.print("Zg= "+ String(zg)+"\n");
  (mq)?Serial.println("SMOKE!"):Serial.println("Safe");


  
  // Set the fields with the values
    ThingSpeak.setField(1,beatsrx);
    ThingSpeak.setField(2,bpmavgrx);
    ThingSpeak.setField(3,pressurerx);
    ThingSpeak.setField(4,temperaturerx);
    ThingSpeak.setField(5,altituderx);
    ThingSpeak.setField(6,mq);
 
 /* Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  pieces of information in a channel.  Here, we write to field 1,2,3,4,5,6. */
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    if(x == 200){ // If the function returns 200 it indicates a successful channel update
      Serial.println("Channel update successful.");
    }
    else{
      // If the function fails to update the channel, corresponding error code value is provided
      Serial.println("Problem updating channel. HTTP error code " + String(x)); 
    }  
}
