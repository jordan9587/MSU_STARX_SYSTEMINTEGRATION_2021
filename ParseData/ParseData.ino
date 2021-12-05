const int NUMBER_OF_FIELDS = 5; // how many comma separated fields we expect
int fieldIndex = 0;            // the current field being received
double dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
double values[NUMBER_OF_FIELDS];
int sign[NUMBER_OF_FIELDS];
void setup()
{
  Serial.begin(9600); // Initialize serial port to send and receive at 9600 baud
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }
}
void serialEvent()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    //Serial.println(ch);
    if(ch >= '0' && ch <= '9')
    {
      dummy[fieldIndex] = (dummy[fieldIndex] * 10) + (ch - '0');
      //Serial.println(dummy[fieldIndex]);
    }
    else if(ch == ',')
    {
      if(fieldIndex < (NUMBER_OF_FIELDS - 1))
        fieldIndex++;
    }
    else if(ch == '-')
      sign[fieldIndex] = -1;
    else
    {
      for(int i = 0; i < NUMBER_OF_FIELDS; i++)
      {
        values[i] = dummy[i]*sign[i];
        Serial.print(dummy[i]); Serial.print(" "); Serial.print(sign[i]); Serial.print(" ");
        Serial.println(values[i]);
        dummy[i] = 0;
        sign[i] = 1;
      }
      fieldIndex = 0;
      digitalWrite(26,values[0]);
      digitalWrite(27,values[1]); 
    } 
  }
}
void loop()
{
//  if((digitalRead(26) == 1 && digitalRead(27) == 1) || (digitalRead(26) == 0 && digitalRead(27) == 0))
//      {
//        Serial.println("Stop");
//      }
//      else
//      {
//        Serial.println("Auto");
//      }
}
