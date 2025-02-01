// Measure distance with TOF10120 sensor
// by Makerguides
#include "Wire.h"
   uint8_t rxPin = 34;
    uint8_t txPin = 15;

int get_distance(int addr) {
  unsigned short dist = 0;
  Wire.beginTransmission(addr);
  Wire.write(0);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(addr, 2);
  if (Wire.available() != 2)
    return -1;
  dist = Wire.read() << 8;
  dist |= Wire.read();
  return dist;
}
void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
 //  Serial2.begin(9600, SERIAL_8N1, rxPin, txPin);
// Serial2.write("s7-164#");
   //Serial2.write("s7-168#");
   // Serial2.write("s7-100#");
   //int dist = get_distance(0x32);
  //  Scanner ();
}

void loop() {
  //default address 
 // Serial2.write("s7-164#");
  int dist1 = get_distance(0x52);
  Serial.print("distance1:");
  Serial.println(dist1);
   delay(200);

//  // Serial2.write("s7-168#");
  int dist2 = get_distance(0x54);
  Serial.print("--Distance2:");
  Serial.println(dist2);
  delay(200);

  // Serial2.write("s7-100#");
  int dist3 = get_distance(0x32);
  Serial.print("--Distance3:");
  Serial.println(dist3);
  // delay(200);
}