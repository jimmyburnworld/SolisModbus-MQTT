/*
jimmyburnworld, October 2022
SolisModbus-MQTT Bridge using ESP8266 based controller
Uses ModbusMaster and SoftwareSerial libraries for the RS485/ Modbus side of things
ESP8266WiFi and ArduinoMqttCLient for the MQTT side
https://github.com/jimmyburnworld/SolisModbus-MQTT

Uses Solis inverter 'COM' port. Set inverter 'address' to 2 - check communications to BMS and Meter after doing this before proceeding
to connect Arduino.

COM port: 1 = +5Vcc, 2 = 0V com, 3 = 'A' 4 = 'B' - This might vary by model, your responsibility to check before connecting
MAX485 GND needed to be connected to 'G' next to 3.3V pin to work.
Used inverter +5V and 0V to power ESP8266 using 'Vin' and 'G' connections & jumpered to MAX485 Vcc & Gnd

KNOWN ISSUES: Signed 16bit values and 32bit values do not work correctly.
Thee signed 16bit values are unsigned because the ModbusMaster library only returns uint16_t type.
32bit values are due to word/ byte swapping Endian-ness etc.
Only included the variables for this project, feel free to add more or remove
Search for RS485_MODBUS-Hybrid-BACoghlan-201811228-1854.pdf for protocol description and registers
*/

#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>
 #include <SoftwareSerial.h>
 // receivePin, transmitPin, inverse_logic, bufSize, isrBufSize
 // connect RX to D6 (ESP2866 D6), TX to D7 (ESP2866 D7)
 SoftwareSerial S(12, 13);

/*!
Use a MAX485 compatible transceiver, bridge DE and RE_NEG and connect to D5 on ESP8266
*/
#define MAX485_DE      14

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}

//*MQTT Initialisation here
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//IP Address for MQTT Broker here. Will work with local name however more reliable with static IP
const char broker[] = //"xxx.xxx.xxx.xxx"; //"core-mosquitto";
int        port     = 1883; //your port here
int count = 0;

long RegAddress[32]; 
int RegWords[32];
float RegScalar[32];
float RegValue[32];
uint8_t RegResult[32];
int RegNo;

// Initialise character arrays with MQTT topic names
char *RegName[] = {"S_IPGen",
"Battery/Solis/S_DV1",
"Battery/Solis/S_DC1",
"Battery/Solis/S_DV2",
"Battery/Solis/S_DC2",
"Battery/Solis/S_DV3",
"Battery/Solis/S_DC3",
"Battery/Solis/S_DV4",
"Battery/Solis/S_DC4",
"Battery/Solis/S_TDOut",
"Battery/Solis/S_DBVol",
"Battery/Solis/S_IT",
"Battery/Solis/S_GF",
"Battery/Solis/S_MAPow",
"Battery/Solis/S_BSVol",
"Battery/Solis/S_BSCur",
"Battery/Solis/S_BSSOC",
"Battery/Solis/S_BSSOH",
"Battery/Solis/S_BVBMS",
"Battery/Solis/S_BCBMS",
"Battery/Solis/S_HLPow",
"Battery/Solis/S_BP",
"Battery/Solis/S_BCTod",
"Battery/Solis/S_BCYes",
"Battery/Solis/S_GITod",
"Battery/Solis/S_GIYes",
"Battery/Solis/S_HLTod",
"Battery/Solis/S_HLYes"};

void setup()
{
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  S.begin(9600, SWSERIAL_8N1);
  
  //Serial Monitor
  Serial.begin(57600);

  // Modbus slave ID 2
  node.begin(2, S);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  //WiFi Connection here
  {
  WiFi.begin("HomeAut", "SfieldHomeAut2021!");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  RegAddress[0] = 33035;
  RegAddress[1] = 33049;
  RegAddress[2] = 33050;
  RegAddress[3] = 33051;
  RegAddress[4] = 33052;
  RegAddress[5] = 33053;
  RegAddress[6] = 33054;
  RegAddress[7] = 33055;
  RegAddress[8] = 33056;
  RegAddress[9] = 33057;
  RegAddress[10] = 33071;
  RegAddress[11] = 33093;
  RegAddress[12] = 33094;
  RegAddress[13] = 33130;
  RegAddress[14] = 33133;
  RegAddress[15] = 33134;
  RegAddress[16] = 33139;
  RegAddress[17] = 33140;
  RegAddress[18] = 33141;
  RegAddress[19] = 33142;
  RegAddress[20] = 33147;
  RegAddress[21] = 33149;
  RegAddress[22] = 33163;
  RegAddress[23] = 33164;
  RegAddress[24] = 33171;
  RegAddress[25] = 33172;
  RegAddress[26] = 33179;
  RegAddress[27] = 33180;

  RegWords[0] = 1;
  RegWords[1] = 1;
  RegWords[2] = 1;
  RegWords[3] = 1;
  RegWords[4] = 1;
  RegWords[5] = 1;
  RegWords[6] = 1;
  RegWords[7] = 1;
  RegWords[8] = 1;
  RegWords[9] = 2;
  RegWords[10] = 1;
  RegWords[11] = 1;
  RegWords[12] = 1;
  RegWords[13] = 2;
  RegWords[14] = 1;
  RegWords[15] = 1;
  RegWords[16] = 1;
  RegWords[17] = 1;
  RegWords[18] = 1;
  RegWords[19] = 1;
  RegWords[20] = 1;
  RegWords[21] = 2;
  RegWords[22] = 1;
  RegWords[23] = 1;
  RegWords[24] = 1;
  RegWords[25] = 1;
  RegWords[26] = 1;
  RegWords[27] = 1;

  RegScalar[0] = 0.1;
  RegScalar[1] = 0.1;
  RegScalar[2] = 0.1;
  RegScalar[3] = 0.1;
  RegScalar[4] = 0.1;
  RegScalar[5] = 0.1;
  RegScalar[6] = 0.1;
  RegScalar[7] = 0.1;
  RegScalar[8] = 0.1;
  RegScalar[9] = 1;
  RegScalar[10] = 0.1;
  RegScalar[11] = 0.1;
  RegScalar[12] = 0.01;
  RegScalar[13] = 1;
  RegScalar[14] = 0.1;
  RegScalar[15] = 0.1;
  RegScalar[16] = 1;
  RegScalar[17] = 1;
  RegScalar[18] = 0.01;
  RegScalar[19] = 0.1;
  RegScalar[20] = 1;
  RegScalar[21] = 1;
  RegScalar[22] = 0.1;
  RegScalar[23] = 0.1;
  RegScalar[24] = 0.1;
  RegScalar[25] = 0.1;
  RegScalar[26] = 0.1;
  RegScalar[27] = 0.1;

}

//MQTT Connection here

  // Each client must have a unique client ID
   mqttClient.setId("SolisInv");

  // You can provide a username and password for authentication
   mqttClient.setUsernamePassword("your user", "your password");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

}

bool state = true;


void loop()
{
  uint8_t result;
  uint16_t data[6];
  String topic;
  
//*MQTT Loop
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  for (RegNo = 0; RegNo <= 27; RegNo++)
    {
    RegResult[RegNo] = 0;

    RegResult[RegNo] = node.readInputRegisters(RegAddress[RegNo],RegWords[RegNo]);

    if (RegResult[RegNo] == node.ku8MBSuccess)
    {
      if (RegWords[RegNo] == 1)
      {
       RegValue[RegNo] = (node.getResponseBuffer(0) * RegScalar[RegNo]); 
      }
      if (RegWords[RegNo] == 2)
      {
       RegValue[RegNo] = ((node.getResponseBuffer(0) + node.getResponseBuffer(1) << 16) * RegScalar[RegNo]);
      }
      
      topic = RegName[RegNo];

      Serial.print("Topic: "); //Used to verify topic string is being created on Serial Monitor
      Serial.println(topic);
      Serial.print("RegValue: ");
      Serial.println(RegValue[RegNo]);  

      mqttClient.beginMessage(topic);
      mqttClient.print(RegValue[RegNo]);
      mqttClient.endMessage();
      yield();
//      delay(100); //Calm things down for monitoring if debugging   
    }
    }
}
