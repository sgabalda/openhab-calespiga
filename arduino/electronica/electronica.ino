#include <DallasTemperature.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <PZEM004Tv30.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <math.h>

#define ARDUINO_CLIENT_ID "arduino_electronica"

// PZEM-004T v3.0 connection and reporting settings
#define CAR_CHARGER_PZEM_RX_PIN A1
#define CAR_CHARGER_PZEM_TX_PIN A2
#define CAR_CHARGER_PZEM_SERIAL_BAUD 9600
#define CAR_CHARGER_PZEM_ADDRESS 0xF8
#define CAR_CHARGER_PZEM_POWER_MQTT_TOPIC "cotxe/carrega/power"
#define CAR_CHARGER_PZEM_ENERGY_MQTT_TOPIC "cotxe/carrega/energy"
#define CAR_CHARGER_PZEM_POWER_CHANGE_THRESHOLD_W 250.0

// temp sensors
#define TOPIC_TEMPERATURE_ELECTRONICS "diposit1/temperature/electronics"
#define TOPIC_TEMPERATURE_BATTERIES "diposit1/temperature/batteries"
#define TOPIC_TEMPERATURE_BATTERIES_CLOSET "diposit1/temperature/batteriescloset"
#define TOPIC_TEMPERATURE_OUTDOOR "diposit1/temperature/outdoor"

// relays
#define TOPIC_FAN_BATTERIES_SET "fan/batteries/set"
#define TOPIC_FAN_BATTERIES_STATUS "fan/batteries/status"
#define TOPIC_FAN_ELECTRONICS_SET "fan/electronics/set"
#define TOPIC_FAN_ELECTRONICS_STATUS "fan/electronics/status"
#define TOPIC_GRID_SET "grid/connection/set"
#define TOPIC_GRID_RELAY_STATUS "grid/connection/action"
#define TOPIC_CAR_CHARGER_SET "cotxe/carrega/set"
#define TOPIC_CAR_CHARGER_STATUS "cotxe/carrega/status"

// PIN SENSORS
#define TOPIC_BATTERY_LEVEL "battery/level/status"
#define TOPIC_GRID_STATUS "grid/connection/status"

// MQTT messages received
#define COMMAND_START "start"
#define COMMAND_STOP "stop"
// MQTT messages sent
#define STATUS_ON "on"
#define STATUS_OFF "off"
#define BATTERY_LEVEL_HIGH "high"
#define BATTERY_LEVEL_MEDIUM "medium"
#define BATTERY_LEVEL_LOW "low"
#define GRID_CONNECTED "on"
#define GRID_DISCONNECTED "off"

// Relay levels
#define RELAY_CAR_CHARGER_ON HIGH
#define RELAY_CAR_CHARGER_OFF LOW
#define RELAY_FAN_ON LOW
#define RELAY_FAN_OFF HIGH
#define RELAY_GRID_ON LOW
#define RELAY_GRID_OFF HIGH

// Used relays
#define RELAY_CAR_CHARGER 7
#define RELAY_FAN_BATTERIES 5
#define RELAY_FAN_ELECTRONICS 6
#define RELAY_GRID 4

// Read pins for battery and grid status
#define PIN_BATTERY_LEVEL_MED 8
#define PIN_BATTERY_LEVEL_LOW 9
#define PIN_GRID_CONNECTED 3

#define PIN_ACTIVE LOW
#define PIN_INACTIVE HIGH

// Data wire is conntected to the Arduino digital pin 4
#define ONE_WIRE_BUS 2

#define MAX_TIME_BETWEEN_ORDERS 3 * 60 * 1000 // 3 minutes

#define TIME_BETWEEN_DATA_PUBLISHED 5 * 1000 // 5 seconds

#define ETHERNET_MAC {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEF}
#define ETHERNET_LOCAL_IP IPAddress(192, 168, 2, 75)
#define MQTT_BROKER_IP IPAddress(192, 168, 2, 114)
#define MQTT_BROKER_PORT 1883

byte mac[] = ETHERNET_MAC;
IPAddress ip = ETHERNET_LOCAL_IP;
IPAddress server = MQTT_BROKER_IP;

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

SoftwareSerial carChargerPzemSerial(CAR_CHARGER_PZEM_RX_PIN, CAR_CHARGER_PZEM_TX_PIN);
PZEM004Tv30 carChargerPzem(static_cast<Stream &>(carChargerPzemSerial), CAR_CHARGER_PZEM_ADDRESS);

EthernetClient ethClient;
PubSubClient client(ethClient);

// last sensor data published
long lastPublished = 0;
unsigned long lastCarChargerPzemPublished = 0;
float previousCarChargerPzemPower = 0.0;
bool hasPreviousCarChargerPzemPower = false;
bool mqttEnabled = true;

long lastOrder = 0; // to store the last time when an order was received

void reconnect() {
  while (!client.connected()) {
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println(F("Connected to MQTT, subscribing"));
      client.subscribe(TOPIC_FAN_BATTERIES_SET);
      client.subscribe(TOPIC_FAN_ELECTRONICS_SET);
      client.subscribe(TOPIC_GRID_SET);
      client.subscribe(TOPIC_CAR_CHARGER_SET);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try in 5s"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// this is not totally required, can be removed if more memory is needed.
// but is useful to identify the 1-wire devices connected.
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7)
      Serial.print(", ");
  }
  Serial.println("");
}

void setup() {

  // Setup default pins to off
  pinMode(RELAY_FAN_BATTERIES, OUTPUT);
  pinMode(RELAY_FAN_ELECTRONICS, OUTPUT);
  pinMode(RELAY_GRID, OUTPUT);
  pinMode(RELAY_CAR_CHARGER, OUTPUT);

  // Setup sensors to input
  pinMode(PIN_BATTERY_LEVEL_LOW, INPUT);
  pinMode(PIN_BATTERY_LEVEL_MED, INPUT);
  pinMode(PIN_GRID_CONNECTED, INPUT);

  Serial.begin(9600);
  sensors.begin();
  carChargerPzemSerial.begin(CAR_CHARGER_PZEM_SERIAL_BAUD);

  // WARNING:  this can be removed if memory is needed, it is useful only when the addresses are
  // identified
  DeviceAddress tempDeviceAddress;
  int numberOfDevices = sensors.getDeviceCount();
  Serial.print(F("Number of devices: "));
  Serial.println(numberOfDevices, DEC);

  for (int i = 0; i < numberOfDevices; i++) {
    Serial.print(i + 1);
    Serial.print(F(" : "));
    sensors.getAddress(tempDeviceAddress, i);
    printAddress(tempDeviceAddress);
  }
  // WARNING remove until here

  turnAllOff();

  if (mqttEnabled) {

    client.setServer(server, MQTT_BROKER_PORT);
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

void readTempSensors() {

  byte sensorBateries[8] = {0x28, 0x2A, 0x27, 0x79, 0x97, 0x10, 0x03, 0xA9};    // 1st on index
  byte sensorExterior[8] = {0x28, 0x07, 0x49, 0x79, 0x97, 0x10, 0x03, 0xE6};    // 2nd on o¡index
  byte sensorElectronica[8] = {0x28, 0xCF, 0x72, 0x79, 0x97, 0x11, 0x03, 0xE8}; // 3rd on index
  byte sensorArmariBat[8] = {0x28, 0xEF, 0x6F, 0x79, 0x97, 0x10, 0x03, 0x0A};   // 4th on index

  sensors.requestTemperatures();

  char cstr[16];
  float tempC;

  tempC = sensors.getTempC(sensorElectronica);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_ELECTRONICS, cstr);
  Serial.print(F("T electronics: "));
  Serial.println(cstr);

  tempC = sensors.getTempC(sensorArmariBat);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_BATTERIES_CLOSET, cstr);
  Serial.print(F("T armari bat: "));
  Serial.println(cstr);

  tempC = sensors.getTempC(sensorBateries);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_BATTERIES, cstr);
  Serial.print(F("T for bat: "));
  Serial.println(cstr);

  tempC = sensors.getTempC(sensorExterior);
  dtostrf(tempC, 4, 2, cstr);
  client.publish(TOPIC_TEMPERATURE_OUTDOOR, cstr);
  Serial.print(F("T outdoor: "));
  Serial.println(cstr);
}

void readAndPublishCarChargerPzem() {
  float carChargerPower = carChargerPzem.power();
  float carChargerEnergy = carChargerPzem.energy();

  if (isnan(carChargerPower) || isnan(carChargerEnergy)) {
    Serial.println(F("PZEM read failed"));
    return;
  }

  bool carChargerPowerChanged =
      hasPreviousCarChargerPzemPower && fabs(carChargerPower - previousCarChargerPzemPower) >=
                                            CAR_CHARGER_PZEM_POWER_CHANGE_THRESHOLD_W;
  previousCarChargerPzemPower = carChargerPower;
  hasPreviousCarChargerPzemPower = true;

  if (millis() - lastCarChargerPzemPublished >= TIME_BETWEEN_DATA_PUBLISHED ||
      carChargerPowerChanged) {
    char powerText[16];
    char energyText[16];

    dtostrf(carChargerPower, 1, 1, powerText);
    dtostrf(carChargerEnergy, 1, 0, energyText);
    client.publish(CAR_CHARGER_PZEM_POWER_MQTT_TOPIC, powerText);
    client.publish(CAR_CHARGER_PZEM_ENERGY_MQTT_TOPIC, energyText);
    Serial.print(F("PZEM power: "));
    Serial.println(powerText);
    Serial.print(F("PZEM energy: "));
    Serial.println(energyText);
    lastCarChargerPzemPublished = millis();
  }
}

void checkMaxTime() {
  /*Serial.print(F("ms since last order: "));
  Serial.println(millis()-lastOrder);*/
  if (millis() - lastOrder > MAX_TIME_BETWEEN_ORDERS) {
    Serial.println(F("Too much time between orders, turning off all relays"));
    turnAllOff();
    lastOrder = millis();
  }
}

// sub callback function
void callback(char *topic, byte *payload, unsigned int length) {

  lastOrder = millis();
  Serial.print(F("[sub: "));
  Serial.print(topic);
  Serial.print(F("] "));
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  Serial.println(message);
  if (strcmp(topic, TOPIC_FAN_BATTERIES_SET) == 0) {
    if (strcmp(message, COMMAND_START) == 0) {
      turnFanBatteries(true);
    } else if (strcmp(message, COMMAND_STOP) == 0) {
      turnFanBatteries(false);
    } else {
      Serial.print(F("-> Error, message not valid. Setting fan batt. off: "));
      Serial.println(message);
      turnFanBatteries(false);
    }
  } else if (strcmp(topic, TOPIC_FAN_ELECTRONICS_SET) == 0) {
    if (strcmp(message, COMMAND_START) == 0) {
      turnFanElectronics(true);
    } else if (strcmp(message, COMMAND_STOP) == 0) {
      turnFanElectronics(false);
    } else {
      Serial.print(F("-> Error, message not valid. Setting fan electr. off: "));
      Serial.println(message);
      turnFanElectronics(false);
    }
  } else if (strcmp(topic, TOPIC_GRID_SET) == 0) {
    if (strcmp(message, COMMAND_START) == 0) {
      turnRelayGrid(true);
    } else if (strcmp(message, COMMAND_STOP) == 0) {
      turnRelayGrid(false);
    } else {
      Serial.print(F("-> Error, message not valid. Setting Grid off: "));
      Serial.println(message);
      turnRelayGrid(false);
    }
  } else if (strcmp(topic, TOPIC_CAR_CHARGER_SET) == 0) {
    if (strcmp(message, COMMAND_START) == 0) {
      turnCarCharger(true);
    } else if (strcmp(message, COMMAND_STOP) == 0) {
      turnCarCharger(false);
    } else {
      Serial.print(F("-> Error, message not valid. Setting Car Charger off: "));
      Serial.println(message);
      turnCarCharger(false);
    }
  } else {
    Serial.println(F("-> Error, topic not valid"));
    turnAllOff();
  }
}

void turnFanElectronics(bool on) {
  if (on) {
    turnRelay(RELAY_FAN_ELECTRONICS, RELAY_FAN_ON);
    client.publish(TOPIC_FAN_ELECTRONICS_STATUS, STATUS_ON);
    Serial.println(F("FAN electronics: ON"));
  } else {
    turnRelay(RELAY_FAN_ELECTRONICS, RELAY_FAN_OFF);
    client.publish(TOPIC_FAN_ELECTRONICS_STATUS, STATUS_OFF);
    Serial.println(F("FAN electronics: OFF"));
  }
}

void turnFanBatteries(bool on) {
  if (on) {
    turnRelay(RELAY_FAN_BATTERIES, RELAY_FAN_ON);
    client.publish(TOPIC_FAN_BATTERIES_STATUS, STATUS_ON);
    Serial.println(F("FAN batteries: ON"));
  } else {
    turnRelay(RELAY_FAN_BATTERIES, RELAY_FAN_OFF);
    client.publish(TOPIC_FAN_BATTERIES_STATUS, STATUS_OFF);
    Serial.println(F("FAN batteries: OFF"));
  }
}

void turnRelayGrid(bool on) {
  if (on) {
    turnRelay(RELAY_GRID, RELAY_GRID_ON);
    client.publish(TOPIC_GRID_RELAY_STATUS, STATUS_ON);
    Serial.println(F("Relay Grid: ON"));
  } else {
    turnRelay(RELAY_GRID, RELAY_GRID_OFF);
    client.publish(TOPIC_GRID_RELAY_STATUS, STATUS_OFF);
    Serial.println(F("Relay Grid: OFF"));
  }
}

void turnCarCharger(bool on) {
  if (on) {
    turnRelay(RELAY_CAR_CHARGER, RELAY_CAR_CHARGER_ON);
    client.publish(TOPIC_CAR_CHARGER_STATUS, STATUS_ON);
    Serial.println(F("Car Charger: ON"));
  } else {
    turnRelay(RELAY_CAR_CHARGER, RELAY_CAR_CHARGER_OFF);
    client.publish(TOPIC_CAR_CHARGER_STATUS, STATUS_OFF);
    Serial.println(F("Car Charger: OFF"));
  }
}

void turnAllOff() {
  turnCarCharger(false);
  turnFanElectronics(false);
  turnFanBatteries(false);
  turnRelayGrid(false);
  Serial.println(F("All relays to Off"));
}

void turnRelay(int relay, int status) {
  digitalWrite(relay, status);
  delay(100);
}

void readBatteryAndGridSensors() {
  int pinValue = digitalRead(PIN_BATTERY_LEVEL_LOW);
  if (pinValue == PIN_ACTIVE) {
    Serial.println(F("Battery level is LOW"));
    client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_LOW);
  } else {
    pinValue = digitalRead(PIN_BATTERY_LEVEL_MED);
    if (pinValue == PIN_ACTIVE) {
      Serial.println(F("Battery level is MEDIUM"));
      client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_MEDIUM);
    } else {
      Serial.println(F("Battery level is HIGH"));
      client.publish(TOPIC_BATTERY_LEVEL, BATTERY_LEVEL_HIGH);
    }
  }
  pinValue = digitalRead(PIN_GRID_CONNECTED);
  if (pinValue == PIN_ACTIVE) {
    Serial.println(F("Grid is connected"));
    client.publish(TOPIC_GRID_STATUS, GRID_CONNECTED);
  } else {
    Serial.println(F("Grid is NOT connected"));
    client.publish(TOPIC_GRID_STATUS, GRID_DISCONNECTED);
  }
}

void loop() {
  if (mqttEnabled) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
  readAndPublishCarChargerPzem();
  if (millis() - lastPublished > TIME_BETWEEN_DATA_PUBLISHED) {
    readTempSensors();
    readBatteryAndGridSensors();
    lastPublished = millis();
  }
  checkMaxTime();
  delay(100);
}