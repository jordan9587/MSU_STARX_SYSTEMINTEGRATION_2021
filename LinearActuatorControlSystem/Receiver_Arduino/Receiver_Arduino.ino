#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

union Onion
{
    uint8_t     fBytes[sizeof( float )];
    float       fValue;
};

Onion flt;

void setup( void )
{
    Serial.begin(9600);
    while( !Serial );
    
}//setup

void loop( void )
{
    byte
        ch, 
        idx;
    bool
        done;
        
    if( Serial.available() > 0 )
    {
        if( Serial.read() == '>' )
        {
            done = false;
            idx = 0;
            while( !done )
            {
                if( Serial.available() > 0 )
                {
                    ch = Serial.read();
                    if( ch == '<' )
                        done = true;
                    else
                    {
                        if( idx < sizeof( float ) )
                            flt.fBytes[idx++] = ch;
                        
                    }//else
                
                }//if
                   
            }//while

            Serial.print( "Float value received: " ); Serial.println( flt.fValue, 4 );
            
        }//if
        
    }//if
    
}//loop
