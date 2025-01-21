#include <Ethernet.h>
#include <PubSubClient.h>
#include <MemoryFree.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARDUINO_CLIENT_ID "arduino_diposit1"
#define TOPIC_MEASUREMENTS_TANK_PERCENT "diposit1/percentage"
#define TOPIC_MEASUREMENTS_TANK_LITERS "diposit1/liters"
#define TOPIC_MEASUREMENTS_TANK_HEIGHT "diposit1/height"
#define TOPIC_MEMORY "diposit1/memory"
#define TOPIC_TEMPERATURE_ELECTRONICS "diposit1/temperature/electronics"
#define TOPIC_TEMPERATURE_BATTERIES "diposit1/temperature/batteries"
#define TOPIC_TEMPERATURE_BATTERIES_CLOSET "diposit1/temperature/batteriescloset"
#define TOPIC_TEMPERATURE_OUTDOOR "diposit1/temperature/outdoor"

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

EthernetClient ethClient;
PubSubClient client(ethClient);

double V0 = 0.52; //Volts without liquid
double V1 = 2.35; //Volts with h1 of liquid
double h1 = 1.32; // height at V1
double level, Vout, aux, i, result;
double area = 5.20; //sq meters of tank
int previousLiters = 0;
int maxHeightCm = 180;
bool mqttEnabled = true;

void reconnect() {
  while (!client.connected()) {
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println("connected");
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try in 5s"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//TODO to be removed when the addresses are identified
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


  if(mqttEnabled){
    byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEF };
    IPAddress ip(192, 168, 2, 75);
    IPAddress server(192, 168, 2, 114);

    client.setServer(server, 1883);
    //client.setCallback(callback);

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

void loop()
{
  if(mqttEnabled){
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
  measureAndPublishTankLevel();
  readTempSensors();
  reportMemory();
  delay(4000);
}