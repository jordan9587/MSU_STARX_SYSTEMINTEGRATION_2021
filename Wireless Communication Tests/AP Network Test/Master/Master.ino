/*
  WiFi AP mode

  This sketch creates an access point without internet
  that other devices can connect to. Once connected, 
  they can access a web server hosted on the board, and
  read analog EMG value.
  
*/
#include <SPI.h> 
#include <WiFiNINA.h>
#include "login_credentials.h" 

// Enter SSID and password in login_credentials library file.
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(23);

boolean alreadyConnected = false; // whether or not the client was connected previously

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

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 23
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
  Serial.println("AP setup successful.");
}


void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status()) 
  {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) 
    {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();   // listen for incoming clients
  if (client) 
  {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    // loop while the client's connected
    while (client.connected()) 
    {
      if (!alreadyConnected) 
      {
        // clead out the input buffer:
        client.flush();
        Serial.println("We have a new client");
        client.println("Hello, client!");
        alreadyConnected = true;
      }
  
      if (client.available() > 0) 
      {
        // read the bytes incoming from the client:
        char c = client.read();
        // echo the bytes back to the client:
        server.write(c);
        // echo the bytes to the server as well:
        Serial.write(c);
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
