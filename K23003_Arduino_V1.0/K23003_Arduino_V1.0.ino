#include <Wire.h>


#define LED_PIN   A7  // I2C Playground LED
#define INT_PIN   4  // IO Expander Interrupt
#define OS_PIN    12  // LM75 Alarm
#define WP_PIN    10  // EEPROM Write Protection


void setup() 
{
  byte error, address;

  // Initialize the IO pins
  pinMode(A7, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  pinMode(OS_PIN, INPUT);
  pinMode(WP_PIN, OUTPUT);


  Wire.begin();

  // Init Serial
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nK23003 Address Scanner");

  // Scan for I2C Addresses
  for (address = 0; address < 127; address++) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) 
    {
      Serial.print("\nI2C Address found: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
    }
  }
}


void loop() 
{
  if(digitalRead(INT_PIN) == 0)
  {
    digitalWrite(A7, !digitalRead(A7));
  }
  

  //delay(10);  // 10ms Delay
}