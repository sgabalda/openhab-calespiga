#include <Ethernet.h>
#include <PubSubClient.h>
#include <MemoryFree.h>

#define ARDUINO_CLIENT_ID "arduino_diposit1"
#define TOPIC_MEASUREMENTS_TANK_PERCENT "diposit1/percentage"
#define TOPIC_MEASUREMENTS_TANK_LITERS "diposit1/liters"
#define TOPIC_MEASUREMENTS_TANK_HEIGHT "diposit1/height"
#define TOPIC_MEMORY "diposit1/memory"

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

void setup()
{

  Serial.begin(9600);

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
  reportMemory();
  delay(4000);
}