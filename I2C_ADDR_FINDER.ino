// Github: https://github.com/pavan-vl/Pilot-Vitals-Monitoring-Jacket
// LinkedIn: https://www.linkedin.com/in/pavan-v-l-2b1986266/

#include <Wire.h> // For I2C Communication

void setup()
{
  Wire.begin(); // Initiate I2C
  Serial.begin(115200); 
  while (!Serial); 
  Serial.println("\nI2C address Scan");
}

void loop()
{
  byte error, addr; // To hold value of error code and I2C address respectively
  int devcount;

  Serial.println("Scanning.........");

  devcount = 0;
  for (addr = 1; addr < 127; addr++ )
  {
    /* Using the return value of
     the Write.endTransmisstion function to check if
     a device acknowledged to the address.*/
    Wire.beginTransmission(addr);
    error = Wire.endTransmission();

    if (error == 0) // No error, address successfully obtained
    {
      Serial.print("I2C device found at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println("  !");
      devcount++;
    }

    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (devcount == 0) // No devices connected 
    Serial.println("No I2C devices connected");
  else
    Serial.println("Done\n");

  delay(2000); 
}
