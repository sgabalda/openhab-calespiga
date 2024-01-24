#include <UIPEthernet.h>
#include <PubSubClient.h>

#define ARDUINO_CLIENT_ID "arduino_diposit1"
#define TOPIC_MEASUREMENTS_TANK_PERCENT "diposit1/percentage"
#define TOPIC_MEASUREMENTS_TANK_LITERS "diposit1/liters"
#define TOPIC_MEASUREMENTS_TANK_HEIGHT "diposit1/height"

EthernetClient ethClient;
PubSubClient client(ethClient);

double V0 = 0.18; //Volts without liquid
double V1 = 1.35; //Volts with h1 of liquid
double h1 = 1.54; // height at V1
double level, Vout, aux, i, result;
double area = 5.29; //sq meters of tank
int previousLiters = 0;
int maxHeightCm = 200;
bool mqttEnabled = true;

void reconnect() {
  while (!client.connected()) {
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try in 5s");
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
      Serial.println("Ethernet shield not found");
      while (true) {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    while (Ethernet.linkStatus() != LinkON) {
      Serial.println("Eth error. 5s retry");
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

  Serial.print("\n\nVoltaje: ");
  Serial.print(Vout);
  Serial.println(" v");
  Serial.print("Nivel: ");
  Serial.print(level);
  Serial.println(" m");
  
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
  Serial.print("Percentage: ");
  Serial.print(percentage);
  Serial.println(" %");
  Serial.print("liters: ");
  Serial.print(liters);
  Serial.println(" L");
  Serial.print("Height: ");
  Serial.print(heightCm);
  Serial.println(" cm");

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
  delay(4000);
}