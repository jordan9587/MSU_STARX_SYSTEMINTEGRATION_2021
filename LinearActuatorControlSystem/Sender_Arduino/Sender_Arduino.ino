#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;
floar f = 23.2;
byte buf[4];
void setup()
{
  Serial.begin(9600); //bit transfer rate (Bd) between PC and UNO
  pinMode(LED_BUILTIN,OUTPUT);
  if (!mpu.begin()) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  
}

void loop()
{
  SUART.write(0x12);  //sync pattern
  SUART.write(0x34);
  byte *ptr = (byte*)&data;
  for (byte i = 0; i < 8; i++, ptr++)
  {
    byte m = *ptr;
    if (m < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(m, HEX);
    SUART.write(m);
    CHKSUM += m;
  }
  CHKSUM = CHKSUM + 0x12 + 0x34;
  SUART.write(CHKSUM);
  Serial.println();
  Serial.print("CHKSUM: "); Serial.println(CHKSUM, HEX);
  CHKSUM = 0;
  Serial.println("==================");
  delay(1000);
}
