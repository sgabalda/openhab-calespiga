#include <Ethernet.h>
#include <PubSubClient.h>
#include <MemoryFree.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARDUINO_CLIENT_ID "arduino_diposit1"

//tank sensor
#define TOPIC_MEASUREMENTS_TANK_PERCENT "diposit1/percentage"
#define TOPIC_MEASUREMENTS_TANK_LITERS "diposit1/liters"
#define TOPIC_MEASUREMENTS_TANK_HEIGHT "diposit1/height"
#define TOPIC_MEMORY "diposit1/memory"

//temp sensors
#define TOPIC_TEMPERATURE_ELECTRONICS "diposit1/temperature/electronics"
#define TOPIC_TEMPERATURE_BATTERIES "diposit1/temperature/batteries"
#define TOPIC_TEMPERATURE_BATTERIES_CLOSET "diposit1/temperature/batteriescloset"
#define TOPIC_TEMPERATURE_OUTDOOR "diposit1/temperature/outdoor"

//relays
#define TOPIC_FAN_BATTERIES_SET "fan/batteries/set"
#define TOPIC_FAN_BATTERIES_STATUS "fan/batteries/status"
#define TOPIC_FAN_ELECTRONICS_SET "fan/electronics/set"
#define TOPIC_FAN_ELECTRONICS_STATUS "fan/electronics/status"
#define TOPIC_TANK1_PUMP_SET "diposit1/pump/set"
#define TOPIC_TANK1_PUMP_STATUS "diposit1/pump/status"

//PIN SENSORS
#define TOPIC_BATTERY_LEVEL "battery/level/status"

//MQTT messages received
#define COMMAND_START "start"
#define COMMAND_STOP "stop"
//MQTT messages sent
#define STATUS_ON "on"
#define STATUS_OFF "off"
#define BATTERY_LEVEL_HIGH "high"
#define BATTERY_LEVEL_MEDIUM "medium"
#define BATTERY_LEVEL_LOW "low"

//Relay levels
#define RELAY_PUMP_ON HIGH
#define RELAY_PUMP_OFF LOW
#define RELAY_FAN_ON LOW
#define RELAY_FAN_OFF HIGH

//Used relays
#define RELAY_FAN_BATTERIES 5
#define RELAY_FAN_ELECTRONICS 6
#define RELAY_PUMP 7

//Read pins for battery status
#define PIN_BATTERY_LEVEL_MED 8
#define PIN_BATTERY_LEVEL_LOW 9

#define PIN_ACTIVE LOW
#define PIN_INACTIVE HIGH

// Data wire is conntected to the Arduino digital pin 4
#define ONE_WIRE_BUS 2

#define MAX_TIME_BETWEEN_ORDERS 3*60*1000   // 3 minutes

#define TIME_BETWEEN_DATA_PUBLISHED 5 * 1000 //5 seconds

byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEF };
IPAddress ip(192, 168, 2, 75);
IPAddress server(192, 168, 2, 114);

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

EthernetClient ethClient;
PubSubClient client(ethClient);

//last sensor data published
long lastPublished = 0;

double V0 = 0.52; //Volts without liquid
double V1 = 2.35; //Volts with h1 of liquid
double h1 = 1.32; // height at V1
double level, Vout, aux, i, result;
double area = 5.20; //sq meters of tank
int previousLiters = 0;
int maxHeightCm = 180;
bool mqttEnabled = true;

long lastOrder = 0;     //to store the last time when an order was received

void reconnect() {
  while (!client.connected()) {
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println(F("Connected to MQTT, subscribing"));
      client.subscribe(TOPIC_FAN_BATTERIES_SET);
      client.subscribe(TOPIC_FAN_ELECTRONICS_SET);
      client.subscribe(TOPIC_TANK1_PUMP_SET);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try in 5s"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//this is not totally required, can be removed if more memory is needed.
//but is useful to identify the 1-wire devices connected.
void printAddress(DeviceAddress deviceAddress)
{ 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

void setup()
{

    //Setup default pins to off
  pinMode(RELAY_FAN_BATTERIES, OUTPUT);
  pinMode(RELAY_FAN_ELECTRONICS, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);

  //Setup sensors to input
  pinMode(PIN_BATTERY_LEVEL_LOW, INPUT);
  pinMode(PIN_BATTERY_LEVEL_MED, INPUT);

  Serial.begin(9600);
  sensors.begin();

  //WARNING:  this can be removed if memory is needed, it is useful only when the addresses are identified
  DeviceAddress tempDeviceAddress;
  int numberOfDevices = sensors.getDeviceCount();
  Serial.print(F("Number of devices: "));
  Serial.println(numberOfDevices, DEC);

  for (int i = 0;  i < numberOfDevices;  i++)
  {
    Serial.print(i+1);
    Serial.print(F(" : "));
    sensors.getAddress(tempDeviceAddress, i);
    printAddress(tempDeviceAddress);
  }
  //WARNING remove until here

  turnAllOff();

  if(mqttEnabled){


    client.setServer(server, 1883);
    client.setCallback(callback);

    Ethernet.begin(mac, ip);
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println(F("Ethernet shield not found"));
      while (true) {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    while (Ethernet.linkStatus() == LinkOFF) {
      Serial.println(F("Eth error. 5s retry"));
      delay(5000);
    }
    Serial.println(F("Connected to Ethernet"));
    delay(1500); // Allow hardware to stabilize 1.5 sec
  }

}

void readTempSensors(){

  byte sensorBateries[8] = {0x28, 0x2A, 0x27, 0x79, 0x97, 0x10, 0x03, 0xA9}; //1st on index
  byte sensorExterior[8] = {0x28, 0x07, 0x49, 0x79, 0x97, 0x10, 0x03, 0xE6}; //2nd on o¡index
  byte sensorElectronica[8] = {0x28, 0xCF, 0x72, 0x79, 0x97, 0x11, 0x03, 0xE8}; //3rd on index
  byte sensorArmariBat[8] = {0x28, 0xEF, 0x6F, 0x79, 0x97, 0x10, 0x03, 0x0A};  //4th on index

  sensors.requestTemperatures();

  char cstr[16];
  float tempC;

  tempC = sensors.getTempC(sensorElectronica);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_ELECTRONICS,cstr);
  Serial.print(F("T electronics: "));
  Serial.println(cstr);

  tempC = sensors.getTempC(sensorArmariBat);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_BATTERIES_CLOSET,cstr);
    Serial.print(F("T armari bat: "));
  Serial.println(cstr);

  tempC = sensors.getTempC(sensorBateries);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_BATTERIES,cstr);
    Serial.print(F("T for bat: "));
  Serial.println(cstr);
  
  tempC = sensors.getTempC(sensorExterior);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_OUTDOOR,cstr);
    Serial.print(F("T outdoor: "));
  Serial.println(cstr);

}

double readHeight() 
{
  //Voltaje del Sensor MPC5010DP
  aux=0;
  for(i=0;i<10;i++){
    aux = aux + (float(analogRead(A0))*5.0/1023.0); //v
    delay(5);
  }
  Vout=aux/10.0;

  level  = (Vout - V0) * h1 / (V1 - V0);
  if(level < 0.0){
    level = 0.0;
  }

  Serial.print(F("\n\nVoltaje: "));
  Serial.print(Vout);
  Serial.println(F(" v"));
  Serial.print(F("Nivel: "));
  Serial.print(level);
  Serial.println(F(" m"));
  
  return level;
}

void measureAndPublishTankLevel(){

  double height;
  int percentage = 0;
  int heightCm = 0;
  int liters = 0;
  // Variable to hold checksum
  unsigned char CS;

  // Array to store incoming serial data
  unsigned char data_buffer[4] = {0};

  height = readHeight();

  heightCm = height * 100;
  liters = height * area * 1000;  //current liters measure
  if(100 < abs(liters-previousLiters)){
    previousLiters = liters;
  }else{
    liters = previousLiters;
  }

  liters = (liters / 50) * 50; //round to 25L
  if(maxHeightCm < heightCm){
    maxHeightCm = heightCm;
  }
  percentage = (100.0 * heightCm) / maxHeightCm;

  if(mqttEnabled){
    char cstr[16];
    itoa(percentage, cstr, 10);
    client.publish(TOPIC_MEASUREMENTS_TANK_PERCENT,cstr);
    itoa(liters, cstr, 10);
    client.publish(TOPIC_MEASUREMENTS_TANK_LITERS,cstr);
    itoa(heightCm, cstr, 10);
    client.publish(TOPIC_MEASUREMENTS_TANK_HEIGHT,cstr);
  }
  Serial.print(F("Percentage: "));
  Serial.println(percentage);
  Serial.print(F("liters: "));
  Serial.println(liters);
  Serial.print(F("Height: "));
  Serial.println(heightCm);

}

void reportMemory(){
  int freeMem = freeMemory();
  if(mqttEnabled){
    char cstr[16];
    itoa(freeMem, cstr, 10);
    client.publish(TOPIC_MEMORY,cstr);
  }
  Serial.print(F("Memory free (bytes): "));
  Serial.println(freeMem);
}

void checkMaxTime(){
  /*Serial.print(F("ms since last order: "));
  Serial.println(millis()-lastOrder);*/
  if(millis()-lastOrder > MAX_TIME_BETWEEN_ORDERS){
    Serial.println(F("Too much time between orders, turning off all pumps"));
    turnAllOff();
    lastOrder=millis();
  }
}

// sub callback function
void callback(char* topic, byte* payload, unsigned int length)
{

  lastOrder = millis();
  Serial.print(F("[sub: "));
  Serial.print(topic);
  Serial.print(F("] "));
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  Serial.println(message);
  if (strcmp(topic, TOPIC_FAN_BATTERIES_SET) == 0){
    if (strcmp(message, COMMAND_START) == 0){
      turnFanBatteries(true);
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnFanBatteries(false);
    }else{
      Serial.print(F("-> Error, message not valid. Setting off: "));
      Serial.println(message);
      turnFanBatteries(false);
    }  
  }else if (strcmp(topic, TOPIC_FAN_ELECTRONICS_SET) == 0){
    if (strcmp(message, COMMAND_START) == 0){
      turnFanElectronics(true);
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnFanElectronics(false);
    }else{
      Serial.print(F("-> Error, message not valid. Setting off: "));
      Serial.println(message);
      turnFanElectronics(false);
    }  
  }else if (strcmp(topic, TOPIC_TANK1_PUMP_SET) == 0){
    if (strcmp(message, COMMAND_START) == 0){
      turnPumpTank1(true);
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnPumpTank1(false);
    }else{
      Serial.print(F("-> Error, message not valid. Setting off: "));
      Serial.println(message);
      turnPumpTank1(false);
    }  
  }else{
    Serial.println(F("-> Error, topic not valid"));
    turnAllOff();
  }
}

void turnPumpTank1(bool on){
  if(on){
    turnRelay(RELAY_PUMP, RELAY_PUMP_ON);
    client.publish(TOPIC_TANK1_PUMP_STATUS, STATUS_ON);
    Serial.println(F("PumpTank1: ON"));
  }else{
    turnRelay(RELAY_PUMP, RELAY_PUMP_OFF);
    client.publish(TOPIC_TANK1_PUMP_STATUS, STATUS_OFF);
    Serial.println(F("PumpTank1: OFF"));
  }
}

void turnFanElectronics(bool on){
  if(on){
    turnRelay(RELAY_FAN_ELECTRONICS, RELAY_FAN_ON);
    client.publish(TOPIC_FAN_ELECTRONICS_STATUS, STATUS_ON);
    Serial.println(F("FAN electronics: ON"));
  }else{
    turnRelay(RELAY_FAN_ELECTRONICS, RELAY_FAN_OFF);
    client.publish(TOPIC_FAN_ELECTRONICS_STATUS, STATUS_OFF);
    Serial.println(F("FAN electronics: OFF"));
  }
}

void turnFanBatteries(bool on){
  if(on){
    turnRelay(RELAY_FAN_BATTERIES, RELAY_FAN_ON);
    client.publish(TOPIC_FAN_BATTERIES_STATUS, STATUS_ON);
    Serial.println(F("FAN batteries: ON"));
  }else{
    turnRelay(RELAY_FAN_BATTERIES, RELAY_FAN_OFF);
    client.publish(TOPIC_FAN_BATTERIES_STATUS, STATUS_OFF);
    Serial.println(F("FAN batteries: OFF"));
  }
}

void turnAllOff(){
  
  turnPumpTank1(false);
  turnFanElectronics(false);
  turnFanBatteries(false);
  Serial.println(F("All relays to Off"));
}

void turnRelay(int relay, int status) {
  digitalWrite(relay, status);
  delay(100);
}

void readBatterySensors(){
 int batteryLow = digitalRead(PIN_BATTERY_LEVEL_LOW);
 if(batteryLow == PIN_ACTIVE){
  Serial.println(F("Battery level is LOW"));
  client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_LOW);
 }else{
  batteryLow = digitalRead(PIN_BATTERY_LEVEL_MED);
  if(batteryLow == PIN_ACTIVE){
    Serial.println(F("Battery level is MEDIUM"));
    client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_MEDIUM);
  }else{
    Serial.println(F("Battery level is HIGH"));
    client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_HIGH);
  }
 }
}
void loop()
{
  if(mqttEnabled){
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
  if(millis()-lastPublished > TIME_BETWEEN_DATA_PUBLISHED){
    measureAndPublishTankLevel();
    readTempSensors();
    readBatterySensors();
    reportMemory();
    lastPublished=millis();
  }
  checkMaxTime();
  delay(100);
}