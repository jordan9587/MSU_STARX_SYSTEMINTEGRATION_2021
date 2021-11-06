/*
  Repeating WiFi Web Client

 This sketch connects to a a web server and makes a request
 using a WiFi equipped Arduino board. Sends the value of the analog input pins
 to the host.

 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>

#include "emgToolbox.h"
//#include "login_credentials.h"

/* Please enter your sensitive data in the Secret tab/login_credentials.h */
char ssid[] = "starxnetwork";      // Your network SSID (name)
// Password has to be >= 8 length
char pass[] = "starxtext";  // Your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;         // Your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Initialize the WiFi client library
WiFiClient client;

// Server address:
//char server[] = "http://192.168.4.1/";
byte server[] = {192, 168, 4, 1};
//IPAddress server(192,168,0,1);

unsigned long lastConnectionTime = 0;   // Last time you connected to the server, in milliseconds
const unsigned long postingInterval = 5L;  // Delay between updates, in milliseconds

// Buffer of HTML.
char c;

// Buffer of EMG array
int maxMatrixSize = 220;
double emgArray[220];

// Message being sent to host.
String idEmg = "A: ";
String clientMessage = String() + idEmg;


void setup()
{
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
      Serial.println("Communication with WiFi module failed!");
      // Don't continue
      while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
      Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
      WiFi.lowPowerMode();  // Enable WiFi Low Power Mode
      // Wait 5 seconds for connection:
      delay(5000);
  }
  // You're connected now, so print out the status:
  printWifiStatus();
}

void loop()
{
  /*
  If there's incoming data from the net connection.
  send it out the serial port.  This is for debugging
  purposes only:
  an HTTP request ends with a blank line
  */
  while (client.available())
  {
      c = client.read();
      Serial.write(c);
  }

  /*
  If ten milliseconds have passed since your last connection,
  then connect again and send data:
  */
  if (millis() - lastConnectionTime > postingInterval)
  {
      httpRequest();
  }

}

// This method makes a HTTP connection to the server:
void httpRequest()
{
  // Close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();
  // If there's a successful connection:
  if (client.connect(server, 80))
  {
      // Reset message being sent to host.
      clientMessage = String() + idEmg;
      for (int pointerEmg = 0; pointerEmg <= maxMatrixSize-2; pointerEmg++)
      {
          // Prepare to send clientMessage to host.
          int sensorValue0 = analogRead(A0);
          int sensorValue1 = analogRead(A1);
          int sensorValue2 = analogRead(A2);
          if (pointerEmg == maxMatrixSize-2)
          {
              // Remove last ", " from clientMessage.
              clientMessage.remove(clientMessage.length() - 1);
              clientMessage.remove(clientMessage.length() - 1);
              client.print(String("Debug: ") + clientMessage);
              client.println();
              // Check the Serial output is correct for client.
              Serial.print("Finished Raw EMG Message: " + clientMessage);
              Serial.print("\n");
              emgFeatureExtraction();
              // Add final delimiter that will be split in server message.
              clientMessage = clientMessage + String("\t");
              // Send clientMessage to host.
              client.print(clientMessage);
              client.println();
              break;
     
          }
          emgArray[pointerEmg] = sensorValue0;
          emgArray[pointerEmg+1] = sensorValue1;
          emgArray[pointerEmg+2] = sensorValue2;
          clientMessage = clientMessage + sensorValue0 + ", " + sensorValue1 + ", " + sensorValue2 + ", ";
      }
     
      // Note the time that the connection was made:
      lastConnectionTime = millis();
  }
  else
  {
      // If you couldn't make a connection:
      Serial.println("connection failed");
  }
}


// Prints all feature extraction results for emg array.
void emgFeatureExtraction()
{
  emgToolbox toolbox(emgArray, maxMatrixSize+1, 0.01);
  // Since we can't initalise function in array define metrics here.
  double ASM = toolbox.ASM();
  double ASS = toolbox.ASS();
  double AAC = toolbox.AAC();
  double ME = toolbox.ME();
  double COV = toolbox.COV();
  double DAMV = toolbox.DAMV();
  double DASDV = toolbox.DASDV();
  double DVARV = toolbox.DVARV();
  double EMAV = toolbox.EMAV();
  double EWL = toolbox.EWL();
  double IEMG = toolbox.IEMG();
  double KURT = toolbox.KURT();
  double LCOV = toolbox.LCOV();
  double LD = toolbox.LD();
  double LDAMV = toolbox.LDAMV();
  double LDASDV = toolbox.LDASDV();
  double LTKEO = toolbox.LTKEO();
  double MFL = toolbox.MFL();
  double MAD = toolbox.MAD();
  double MAV = toolbox.MAV();
  double MSR = toolbox.MSR();
  double MMAV = toolbox.MMAV();
  double MMAV2 = toolbox.MMAV2();
  double MYOP = toolbox.MYOP();
  double RMS = toolbox.RMS();
  double SSI = toolbox.SSI();
  double SKEW = toolbox.SKEW();
  double SSC = toolbox.SSC();
  double SD = toolbox.SD();
  double TM = toolbox.TM();
  double VAR = toolbox.VAR();
  double VAREMG = toolbox.VAREMG();
  double VO = toolbox.VO();
  double WL = toolbox.WL();
  double WA = toolbox.WA();
    
  // Add each metric to array of emg features.
  double emgFeatures[] = {ASM, ASS, AAC, ME,
                          COV, DAMV, DASDV, DVARV, EMAV,
                          EWL, IEMG, KURT, LCOV,
                          LD, LDAMV, LDASDV, LTKEO, MFL,
                          MAD, MAV, MSR, MMAV, MMAV2,
                          MYOP, RMS, SSI, SKEW,
                          SSC, SD, TM, VAR, VAREMG,
                          VO, WL, WA};
  // Format message as comma seperated.
  for (int a=0; a < (sizeof emgFeatures/sizeof emgFeatures[0]); a++)
  {
      double val = emgFeatures[a];
      String SerialData = "";
      SerialData = String(val,5);
      clientMessage = clientMessage + SerialData + String(", ");
  }
    
  // Remove last ", " from clientMessage.
  clientMessage.remove(clientMessage.length() - 1);
  clientMessage.remove(clientMessage.length() - 1);

  // Add end of line to message for server to determine when to print.
  clientMessage = clientMessage + String("\n");
         
  // Check the Serial output is correct for client.
  Serial.print("Finished Metric Message: " + clientMessage);
  Serial.print("\n");
}

void printWifiStatus()
{
  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
