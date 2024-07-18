#include <WiFi.h>  // For utilizing ESP-32 capabilities

/* Reason to connect to an Access Point (WiFi) is that there are some case in which ESP-32 will not provide 
   its MAC address unless its connected an access point and is assigned an IP Adress */
// Replace with your network credentials
const char* ssid = "YourWiFI";
const char* password = "YourPassword";

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi network
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
 
    
  if (WiFi.status() == WL_CONNECTED) {
    // Get and print the MAC address of the ESP32
    String macAddress = WiFi.macAddress();
    Serial.println("ESP32 MAC Address: " + macAddress);    
  } 
  else {
    Serial.println("\nFailed to connect to Wi-Fi");
  }
}
void loop() {
}
