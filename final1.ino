//----------------------------------------Include the NodeMCU ESP8266 Library
//----------------------------------------see here: https://www.youtube.com/watch?v=8jMr94B8iN0 to add NodeMCU ESP12E ESP8266 library and board (ESP8266 Core SDK)
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>                           /* include virtual Serial Port coding */
SoftwareSerial PZEMSerial;   
#include <ModbusMaster.h>                             // Load the (modified) library for modbus communication command codes. Kindly install at our website.
#define MAX485_DE  16                                 // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module to Pin D0 (GPIO 16) Node MCU board
#define MAX485_RE  5 

static uint8_t pzemSlaveAddr = 0x01;                  // Declare the address of device (meter 1) in term of 8 bits.
static uint16_t NewshuntAddr = 0x0003;                // Declare your external shunt value for DC Meter. Default 0x0000 is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
uint16_t _u16ResponseBuffer[8];                       //Buffer đầu ra cho 8 thanh ghi
uint8_t _u8ResponseBuffer[21];
ModbusMaster node;  

float PZEMVoltage = 0;                                /* Declare value for DC voltage */
float PZEMCurrent = 0;                                /* Declare value for DC current*/
float PZEMPower = 0;                                  /* Declare value for DC Power */
float PZEMEnergy = 0;  

unsigned long startMillisPZEM;                        /* start counting time for LCD Display */
unsigned long currentMillisPZEM;                      /* current counting time for LCD Display */
const unsigned long periodPZEM = 1000;  

unsigned long startMillisReadData;                    /* start counting time for data collection */
unsigned long currentMillisReadData;                  /* current counting time for data collection */
const unsigned long periodReadData = 1000;            /* refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second */
int ResetEnergy = 0;                                  /* reset energy function */
int a = 1;
unsigned long startMillis1;   

#define ON_Board_LED 2  
const char* ssid = "Na_No"; //--> Your wifi name or SSID.
const char* password = "tu1denchin"; //--> Your wifi password.
//----------------------------------------

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------

WiFiClientSecure client; //--> Create a WiFiClientSecure object.

String GAS_ID = "AKfycbwH3rlcgP8s8ZfC2z94VXVaktptvAaoVjaYbCB2hV-hoptkjTCC"; //--> spreadsheet script ID

//============================================================================== void setup
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);
  
  WiFi.begin(ssid, password); //--> Connect to your WiFi router
  Serial.println("");
    
  pinMode(ON_Board_LED,OUTPUT); //--> On Board LED port Direction output
  digitalWrite(ON_Board_LED, HIGH); //--> Turn off Led On Board

  //----------------------------------------Wait for connection
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    //----------------------------------------Make the On Board Flashing LED on the process of connecting to the wifi router.
    digitalWrite(ON_Board_LED, LOW);
    delay(250);
    digitalWrite(ON_Board_LED, HIGH);
    delay(250);
    //----------------------------------------
  }
  //----------------------------------------
  digitalWrite(ON_Board_LED, HIGH); //--> Turn off the On Board LED when it is connected to the wifi router.
  //----------------------------------------If successfully connected to the wifi router, the IP Address that will be visited is displayed in the serial monitor
  Serial.println("");
  Serial.print("Successfully connected to : ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  //----------------------------------------

  client.setInsecure();
  startMillis1 = millis();

  
  PZEMSerial.begin(9600, SWSERIAL_8N2, 4, 0);           // 4 = Rx/R0/ GPIO 4 (D2) & 0 = Tx/DI/ GPIO 0 (D3) on NodeMCU
  
  /* 1- PZEM-017 DC Energy Meter */

  startMillisPZEM = millis();                           /* Start counting time for run code */
  pinMode(MAX485_RE, OUTPUT);                           /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE, OUTPUT);                           /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  digitalWrite(MAX485_RE, 0);                           /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE, 0);                           /* Arduino create output signal for pin DE as LOW (no output)*/
  // both pins no output means the converter is in communication signal receiving mode
  node.preTransmission(preTransmission);                // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);
  node.begin(pzemSlaveAddr, PZEMSerial);
  delay(1000);                                          /* after everything done, wait for 1 second */

  /* 2 - Data submission to Blynk Server  */

  startMillisReadData = millis();   

}
uint8_t add = 0x01;
void send_request()
{
  /* 1- PZEM-017 DC Energy Meter */
  Serial.print("\r\nBegin TEST ADDR "); Serial.print(add);

  preTransmission();                                                                                /* trigger transmission mode*/
  int l = 0;
  l = PZEMSerial.write(0x01);                                                                      /* these whole process code sequence refer to manual*/
  l = PZEMSerial.write(0x04);
  l = PZEMSerial.write(0x00);
  l = PZEMSerial.write(0x00);
  l = PZEMSerial.write(0x00);
  l = PZEMSerial.write(0x08);
  l = PZEMSerial.write(0xF1);
  l = PZEMSerial.write(0xCC);
  postTransmission();
  uint8_t u8BytesLeft = 21;
  uint8_t u8BufferIndex = 0;
  startMillisReadData = millis();
  while (u8BytesLeft)
  {
    if (PZEMSerial.available())
    {

      _u8ResponseBuffer[u8BufferIndex] = PZEMSerial.read();
      //      Serial.print("\r\n"); Serial.print(_u8ResponseBuffer[u8BufferIndex],HEX);
      u8BufferIndex += 1;
      u8BytesLeft--;
    }
    if ((millis() - startMillisReadData) > 2000) {
      break;
    }
  }
  if (u8BytesLeft == 0) {
    for (int i = 0 ; i < 8; i++) {
      _u16ResponseBuffer[i] |= (uint16_t)(_u8ResponseBuffer[3 + i * 2] << 8);
      _u16ResponseBuffer[i] |= (uint16_t)(_u8ResponseBuffer[4 + i * 2]);
    }
  }

}

void loop() {
    
  send_request();
  uint32_t tempdouble = 0x00000000;                                                           /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
  PZEMVoltage = _u16ResponseBuffer[0] / 100.0;                                       /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
  // 0x0000 to 0x0008 are the register address of the measurement value
  PZEMCurrent = _u16ResponseBuffer[1] / 100.0;                                       /* get the 16bit value for the current value, divide it by 100 (as per manual) */
  Serial.print(tempdouble); Serial.println(" hi ");
  tempdouble =  (_u16ResponseBuffer[3] << 16) + _u16ResponseBuffer[2];      /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
  PZEMPower = tempdouble/10.0;                                                              /* Divide the value by 10 to get actual power value (as per manual) */

  tempdouble =  (_u16ResponseBuffer[5] << 16) + _u16ResponseBuffer[4];      /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
  PZEMEnergy = tempdouble;
  Serial.print("\r\nVdc : "); Serial.print(PZEMVoltage); Serial.println(" V ");
  Serial.print("\r\nIdc : "); Serial.print(PZEMCurrent); Serial.println(" A ");
  Serial.print("\r\nPower : "); Serial.print(tempdouble); Serial.println(" W ");
  Serial.print("\r\nEnergy : "); Serial.print(PZEMEnergy); Serial.println(" Wh ");
 
 
  delay(1000);
  Serial.println("==========");
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }
   String url = "/macros/s/" + GAS_ID + "/exec?Voltage=" + String(PZEMVoltage) + "&Current=" + String(PZEMCurrent)+ "&Energy=" + String(PZEMPower)+ "&Power=" + String(PZEMEnergy);
  Serial.print("requesting URL: ");
  Serial.println(url);
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
         "Host: " + host + "\r\n" +
         "User-Agent: BuildFailureDetectorESP8266\r\n" +
         "Connection: close\r\n\r\n");

  Serial.println("request sent");
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if (line.startsWith("{\"state\":\"success\"")) {
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.print("reply was : ");
  Serial.println(line);
  Serial.println("closing connection");
  Serial.println("==========");
  Serial.println();
  
}
void preTransmission()
{

  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
  delay(5);
}
void postTransmission()                                                                                   /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */

  //    delay(3);                                                                                       // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE, 0);                                                                     /* put RE Pin to low*/
  digitalWrite(MAX485_DE, 0);                                                                     /* put DE Pin to low*/

}
