/*
  Turns on the three relays for the heater
*/
#include <UIPEthernet.h>
#include <PubSubClient.h>
#include <MemoryFree.h>

#define ARDUINO_CLIENT_ID "arduino_calentador"                     // Client ID for Arduino pub/sub
#define SUB_CALENTADOR "arduino_calentador/potencia/set"
#define PUB_CALENTADOR "arduino_calentador/potencia/status"
#define PUB_TERMOSTAT "arduino_calentador/termostat/status"
#define PUB_TERMOSTAT "arduino_calentador/termostat/status"
#define TOPIC_MEMORY "arduino_calentador/memory"
#define MAX_TIME_BETWEEN_ORDERS 3*60*1000   // 3 minutes

// Networking details
byte mac[]    = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };  // Ethernet shield (W5100) MAC address
IPAddress ip(192, 168, 2, 100);                           // Ethernet shield (W5100) IP address
IPAddress server(192, 168, 2, 114);                       // MTTQ server IP address

EthernetClient ethClient;
PubSubClient client(ethClient);

//Relay config
int RELAY_1 = 5;
int RELAY_2 = 6;
int RELAY_3 = 7;
int OPTO_READ = 3;
byte optoHOT = HIGH;
byte optoCOLD = LOW;   //HIGH is when the optocoupler does receive 220V
byte relayON = LOW;
byte relayOFF = HIGH;

long lastOrder = 0;     //to store the last time when an order was received

void setup() {
  //Setup default pins to off
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(OPTO_READ, INPUT);
  calentadorOff();

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println(F("Calentador Arduino starting")); 
  
  
  // MTTQ parameters
  client.setServer(server, 1883);
  client.setCallback(callback);
 
  // Ethernet shield configuration
  Ethernet.begin(mac, ip);

// Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("Ethernet shield not found, can't run."));
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("Ethernet cable not connected."));
  }
 
  delay(1500); // Allow hardware to stabilize 1.5 sec

  Serial.print(F("IP is at "));
  Serial.println(Ethernet.localIP());

  lastOrder = millis();
  
}

void reportMemory(){
  int freeMem = freeMemory();

  char cstr[16];
  itoa(freeMem, cstr, 10);
  client.publish(TOPIC_MEMORY,cstr);

  Serial.print(F("Memory free (bytes): "));
  Serial.println(freeMem);
}

void loop() {
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  checkMaxTime();
  checkOptStatus();
  reportMemory();
  delay(2000);
}

void checkOptStatus(){
  Serial.println(F("Checking optocuopler"));
  byte optStatus = digitalRead(OPTO_READ);
  if(optStatus == optoHOT){
    Serial.println(F("HOT, sending status"));
    client.publish(PUB_TERMOSTAT, "HOT");
  }else{
    Serial.println(F("COLD, sending status"));
    client.publish(PUB_TERMOSTAT, "COLD");
  }
}

void checkMaxTime(){
  Serial.print(F("ms since last order: "));
  Serial.println(millis()-lastOrder);
  if(millis()-lastOrder > MAX_TIME_BETWEEN_ORDERS){
    Serial.println(F("Too much time between orders, turning off calentador:"));
    calentadorOff();
    lastOrder=millis();
  }
}

void reconnect()
{

  // Loop until reconnected
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection ... "));
    // Attempt to connect
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println(F("connected"));
      // (re)subscribe
      client.subscribe(SUB_CALENTADOR);
    } else {
      Serial.print(F("Connection failed, state: "));
      Serial.print(client.state());
      Serial.println(F(", retrying in 5 seconds"));
      checkMaxTime();
      checkOptStatus();
      delay(5000); // Wait 5 seconds before retrying
    }
    Serial.println(F("done 1st try to connect"));
  }
  Serial.println(F("Client connected"));
}

// sub callback function
void callback(char* topic, byte* payload, unsigned int length)
{
  lastOrder = millis();
  Serial.print("[sub: ");
  Serial.print(topic);
  Serial.print("] ");
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  Serial.println(message);
  if (strcmp(message, "0") == 0){
      calentadorOff();
  }else if (strcmp(message, "500") == 0){
      calentador500W();
  }else if (strcmp(message, "1000") == 0){
      calentador1000W();
  }else if (strcmp(message, "2000") == 0){
      calentador2000W();
  }else{
      Serial.println(F("Error, message not valid:"));
      calentadorOff();
  }
}


void calentadorOff(){
  turnRelay(RELAY_3, relayOFF); //turn off in the opposite order, thus relay_1 does not receive a lot of current
  turnRelay(RELAY_2, relayOFF);
  turnRelay(RELAY_1, relayOFF);
  client.publish(PUB_CALENTADOR, "OFF");
  Serial.println(F("Calentador: Off"));
}

void calentador500W(){
  turnRelay(RELAY_1, relayON);
  turnRelay(RELAY_2, relayOFF);
  turnRelay(RELAY_3, relayOFF);
  client.publish(PUB_CALENTADOR, "500");
  Serial.println(F("Calentador: 500"));
}

void calentador1000W(){
  turnRelay(RELAY_1, relayON);
  turnRelay(RELAY_2, relayON);
  turnRelay(RELAY_3, relayOFF);
  client.publish(PUB_CALENTADOR, "1000");
  Serial.println(F("Calentador: 1000"));
}

void calentador2000W(){
  turnRelay(RELAY_1, relayON);
  turnRelay(RELAY_2, relayON);
  turnRelay(RELAY_3, relayON);
  client.publish(PUB_CALENTADOR, "2000");
  Serial.println(F("Calentador: 2000"));
}

void turnRelay(int relay, int status) {
  digitalWrite(relay, status);
  delay(330);
}
