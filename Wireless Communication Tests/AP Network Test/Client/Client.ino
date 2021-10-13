/*
  Repeating WiFi Web Client

 This sketch connects to a a web server and makes a request
 using a WiFi equipped Arduino board. Sends the value of the analog input pins
 to the host.

 */

#include <SPI.h>
#include <WiFiNINA.h>

//#include "emgToolbox.h"
//#include "login_credentials.h"

///////please enter your sensitive data in the Secret tab/login_credentials.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Initialize the WiFi client library
WiFiClient client;

// server address:
//char server[] = "http://192.168.4.1/";
byte server[] = {192, 168, 4, 1};
//IPAddress server(192,168,0,1);

unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L; // delay between updates, in milliseconds

/* User Input */
int numberLine = 0;
int userInputMessageCounter = 0;
bool printOutputBool = false;

// Buffer of HTML.
char c;
// Boolean for buffer newline.
boolean currentLineIsBlank = true;

// Buffer of EMG array
double emgArray[99];

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 5 seconds for connection:
    delay(5000);
  }
  // you're connected now, so print out the status:
  printWifiStatus();
}

void loop() {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  // an HTTP request ends with a blank line
  currentLineIsBlank = true;
  while (client.available()) {
    c = client.read();
    Serial.write(c);
  }

  // if ten milliseconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    httpRequest();
  }

}

// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();
  //Serial.println("connecting...");
  // if there's a successful connection:
  if (client.connect(server, 80)) {
    // send a standard HTTP response header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println("Refresh: 1");  // refresh the page automatically every 1 sec
    //client.println();
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    
    // output the value of each analog input pin
    int sensorValue0 = analogRead(A0);
    client.println(sensorValue0);
    // emgSensorRead(A0);
    /*
    for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
      int sensorReading = analogRead(analogChannel);
      client.print("analog input ");
      client.print(analogChannel);
      client.print(" is ");
      client.print(sensorReading);
      client.println("<br />");
     }
     */
     
     client.println("</html>");
     // note the time that the connection was made:
     lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}

void emgSensorRead(int pin0)
{
  if (userInputMessageCounter == 0) {
      Serial.println("Please submit anything to console in order to write new emg values.");
      userInputMessageCounter += 1;
  }
  while(Serial.available() || (printOutputBool == true))    // Check if there is any user input
  {       
    // Reset while loop once reaching 100 lines of output. Require new user input to run function
    // again.
    if (numberLine >= 99)
    {
      Serial.readString();
      // emgFeatureExtraction();
      printOutputBool = false;
      numberLine = 0;
      userInputMessageCounter = 0;
      Serial.println("Done");
    }
    else
    {
      // Put functions here you want to repeat after user input.
      // Read analog pins value.
      float sensorValue0 = analogRead(pin0);
      emgArray[numberLine] = sensorValue0;
      // Print analog pins value to client.
      client.print("EMG Sensor Reading:");
      client.print(sensorValue0);
      client.print(", ");
      
      numberLine += 1;
      printOutputBool = true;
    }
  }
}

/*
// Prints all feature extraction results for emg array.
void emgFeatureExtraction()
{
  emgToolbox toolbox(emgArray, 99, 0.01);
  double emgFeatures[39] = {toolbox.ASM(), toolbox.ASS(), toolbox.AAC(), toolbox.ME(), toolbox.CARD(),
                            toolbox.COV(), toolbox.DAMV(), toolbox.DASDV(), toolbox.DVARV(), toolbox.EMAV(),
                            toolbox.EWL(), toolbox.IEMG(), toolbox.IQR(), toolbox.KURT(), toolbox.LCOV(),
                            toolbox.LD(), toolbox.LDAMV(), toolbox.LDASDV(), toolbox.LTKEO(), toolbox.MFL(),
                            toolbox.MAD(), toolbox.MAV(), toolbox.MSR(), toolbox.MMAV(), toolbox.MMAV2(), 
                            toolbox.MYOP(), toolbox.FZC(), toolbox.RMS(), toolbox.SSI(), toolbox.SKEW(),
                            toolbox.SSC(), toolbox.SD(), toolbox.TM(), toolbox.VAR(), toolbox.VAREMG(),
                            toolbox.VO(), toolbox.WL(), toolbox.WA(), toolbox.ZC()}; 
   client.print("\nEMG Feature Extractions:");
   for (int a = 0; a < 38; a++)
   {
      client.print(emgFeatures[a]);
      client.print(", ");
   }
}
*/

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
