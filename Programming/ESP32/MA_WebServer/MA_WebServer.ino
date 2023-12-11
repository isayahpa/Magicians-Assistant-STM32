#include <WiFi.h>
#include <HardwareSerial.h>
//#include <SD.h>


//Commands to be sent to STM32
#define BUFFER_SIZE 5000

const String LIGHTS_ON = "light_on";
const String LIGHTS_OFF = "lightoff";
const String SHUFFLE = "shuffle_";
const String SNAP = "picture_";
const String ARCHIDEKT = "archidekt";
const String SD_READ = "read_sd_";
const String SHUTDOWN = "shutdown";

const String STATUS_LIGHTS_ON = "Turning Lights On";
const String STATUS_LIGHTS_OFF = "Turning Lights Off";
const String STATUS_SHUFFLE = "Starting Shuffle";
const String STATUS_SNAP = "Taking a Picture";
const String STATUS_ARCHIDEKT = "Sending Deck to Archidekt";
const String STATUS_SD_READ = "Requesting SD Data";
const String STATUS_SHUTDOWN = "Shutting Down";
const String STATUS_UNKNOWN = "Unknown Action";


// Replace with your network credentials
const char* ssid = "Varsity WiFi";
const char* password = "\0";

// Set web server port number to 80
WiFiServer server(80);
HardwareSerial STMSerialPort(2); // Rx = 16 (Yellow), Tx (Blue) = 17

// Variable to store the HTTP request
String header;
String filename = "";

uint8_t dataBuffer[BUFFER_SIZE] = {};
String pictureDataBuffer = "";
String textDataBuffer = "";

// Client state variables
String status;
bool lightsOn = false;
bool showPreview = false;

// Assign output variables to GPIO pins
const int STM_READY_PIN = 21;
const int LED_PIN = 2;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 20000;

void setup() {
  
  Serial.begin(115200);
  STMSerialPort.begin(115200); // Rx = 16 (Yellow), Tx (Blue) = 17

  pinMode(STM_READY_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  status = "Init";
  lightsOn = false;
  showPreview = false;

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Takes the request and sends it to the STM32
            if (header.indexOf("GET /lights/on/") >= 0) {
              sendCMD(LIGHTS_ON);
            } else if (header.indexOf("GET /lights/off/") >= 0) {
              sendCMD(LIGHTS_OFF);
            } else if (header.indexOf("GET /shuffle/start/") >= 0) {
              sendCMD(SHUFFLE);
            } else if (header.indexOf("GET /camera/snap/") >= 0) {
              sendCMD(SNAP);
            } else if (header.indexOf("GET /archidekt/send/") >= 0) {
              sendCMD(ARCHIDEKT);
            } else if (header.indexOf("GET /shutdown/") >= 0) {
              sendCMD(SHUTDOWN);
            }

            client.println("<!DOCTYPE html>");
            client.println("<head>");
            client.println("<title>The Magician's Assistant</title>");
            client.println("</head>");

            client.println("<body>");
            client.println("<h1>Welcome to the Magician's Assistant!</h1>");
            client.println("<div>");
            client.println("<h3>Lights</h3>");
            if(lightsOn){ // "If lights are on, then display the Off button"
              client.println("<p><a href=\"/lights/off/\"><button class=\"button\">Off</button></a></p>");
            } else {
              client.println("<p><a href=\"/lights/on/\"><button class=\"button\">On</button></a></p>");
            }
            client.println("<h3>Shuffle</h3>");
            client.println("<p><a href=\"/shuffle/start/\"><button class=\"button\">Start</button></a></p>");
            client.println("<h3>Camera</h3>");
            client.println("<p><a href=\"/camera/snap/\"><button class=\"button\">Snap</button></a></p>");
            client.println("<h3>Send to Archidekt</h3>");
            client.println("<p><a href=\"/archidekt/send/\"><button class=\"button\">Send</button></a></p>");
            client.println("<h3>Shutdown</h3>");
            client.println("<p><a href=\"/shutdown/\"><button class=\"button\">Shutdown</button></a></p>");
            client.println("</div>");

            //Preview the photo data when photos are sent
            if(showPreview){
              showPreviewInClient(client);
            }

            client.println("<p>Status: " + status + "</p>");
            client.println("</body>");
            client.println("</html>");
            
            //A redirect to the original IP will help prevent dupe cmds

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

//ESP32's Command Handlers 
void sendCMD(String cmd){

  unsigned long timeBefSend = micros();
  flushSTMRXBuffer(); // Clear the RX buffer in case we need to take data in after the command
  STMSerialPort.print(cmd.c_str()); // Send CMD
  STMSerialPort.flush();
  unsigned long timeAftSend = micros();
  Serial.println("Took " + String(timeAftSend - timeBefSend) + "μs to send CMD.");
  
  if(waitForSTM() == false) { //Wait for STM to execute the command
    Serial.println("Timed out waiting for STM, must be busy..."); 
    return; // Return on timeout so that the client can still get updated
  }

  int nBytesToRead = STMSerialPort.available();
  Serial.println("STM Completed - Received " + String(nBytesToRead) + " bytes of data");
  Serial.println("Updating ESP State");
  // On a successful wait, update state variables
  if(cmd == LIGHTS_ON){
    status = STATUS_LIGHTS_ON;
    lightsOn = true;
  } else if (cmd == LIGHTS_OFF){
    status = STATUS_LIGHTS_OFF;
    lightsOn = false;
  } else if (cmd == SHUFFLE){
    status = STATUS_SHUFFLE;
  } else if (cmd == SNAP){
    status = STATUS_SNAP;
    showPreview = true;
  } else if (cmd == SD_READ){
  } else if (cmd == ARCHIDEKT){
    status = STATUS_ARCHIDEKT;
    int nBytesToRead = STMSerialPort.available();
    readFromSTM(nBytesToRead);
    textDataBuffer = (char*) dataBuffer;
  } else if (cmd == SHUTDOWN) {
    status = STATUS_SHUTDOWN;
  }

}

// Makes the ESP wait for STM to be Ready
//Returns true if the Ready Flag was noticed successfully, False on Timeout
bool waitForSTM(){
  Serial.println("Waiting for STM To Be Ready");
  unsigned long startTime = millis(); 
  while(digitalRead(STM_READY_PIN) != HIGH){
    if(millis() - startTime >= timeoutTime){
      return false;
    }
  }
  return true;
}

void showPreviewInClient(WiFiClient client){
  Serial.println("Reading Base64 Picture Data...");
  //if(client){
    int c;

    client.print("<img src=\"data:image/jpg;base64, ");
    Serial.print("<img src=\"data:image/jpg;base64, ");
    while(STMSerialPort.available() > 0){
      c = STMSerialPort.read();
      client.print((char)c);
      Serial.print((char)c);
    }
    client.print("\" alt=\"Arducam Preview\"/>\n");
    Serial.print("\" alt=\"Arducam Preview\"/>\n");
  //} else {
    //Serial.println("FAILED Show Preview in Client. Client was NULL.");
  //}
  
}

void readFromSTM(int nBytes){
  //dataBuffer = ;
  Serial.println("Reading " + String(nBytes) + " from STM");
    for(int i = 0; i < nBytes; i++){
      dataBuffer[i] = (uint8_t) STMSerialPort.read();
    }

    Serial.println("Raw Data from STM:");
    for(int i = 0; i < nBytes; i++){
      Serial.print(String(dataBuffer[i], HEX));
    }
    Serial.println();
}

void flushSTMRXBuffer(){
  while(STMSerialPort.available()){
    STMSerialPort.read();
  }
}

const char BASE64LOOKUPTABLE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
String hexToBase64(uint8_t *inputBytes, int nBytes){
  Serial.println("Converting to Base64");
  String output = "";

  uint32_t byte0, byte1, byte2;
  uint8_t base64Digit0, base64Digit1, base64Digit2, base64Digit3;
  uint8_t tableIdx0, tableIdx1, tableIdx2, tableIdx3;
  uint32_t threeByteCombo;
  int inputIdx = 0;
  
  while(inputIdx < nBytes-2){
    byte0 = inputBytes[inputIdx];
    byte1 = inputBytes[inputIdx+1];
    byte2 = inputBytes[inputIdx+2];
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits
    
    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;
    
    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = BASE64LOOKUPTABLE[tableIdx2];
    base64Digit3 = BASE64LOOKUPTABLE[tableIdx3];

    output += base64Digit0;
    output += base64Digit1;
    output += base64Digit2;
    output += base64Digit3;

    inputIdx += 3;
  }

  // Padding if necessary
  if((nBytes-1)%3 == 2){
    
    byte0 = inputBytes[nBytes-2];
    byte1 = inputBytes[nBytes-1];
    byte2 = 0; //Will need to pad here
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits
    
    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;
    
    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = BASE64LOOKUPTABLE[tableIdx2];
    base64Digit3 = '=';
    
    output += base64Digit0;
    output += base64Digit1;
    output += base64Digit2;
    output += base64Digit3;
  } else if ((nBytes-1)%3 == 1){
    byte0 = inputBytes[nBytes-1];
    byte1 = 0; //Will need to pad here
    byte2 = 0; 
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits
    
    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;
    
    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = '=';
    base64Digit3 = '=';

    output += base64Digit0;
    output += base64Digit1;
    output += base64Digit2;
    output += base64Digit3;
  }

  Serial.println("Base 64 Data = " + output);
  //strcpy(pictureDataBuffer, output.c_str());
  return output;
}


