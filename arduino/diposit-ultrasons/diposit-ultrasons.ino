
#include <UIPEthernet.h>
#include <PubSubClient.h>
#include <NeoSWSerial.h>

#define ARDUINO_CLIENT_ID "arduino_diposit1"
#define TOPIC_MEASUREMENTS_TANK_PERCENT "diposit1/percentage"
#define TOPIC_MEASUREMENTS_TANK_LITERS "diposit1/liters"

EthernetClient ethClient;
PubSubClient client(ethClient);

NeoSWSerial sensorSerial(7, 8);

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
  byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEF };
  IPAddress ip(192, 168, 2, 75);
  IPAddress server(192, 168, 2, 114);

  Serial.begin(57600);

  sensorSerial.begin(9600);

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
void measureAndPublishTankLevel(){

  // Variable to hold checksum
  unsigned char CS;

  // Array to store incoming serial data
  unsigned char data_buffer[4] = {0};
  
  // Integer to store distance
  int distance = 0;
  float mean = 0;
  int samples = 0;
  int percentage = 0;
  int liters = 0;
  float sum = 0;

  for (int j = 0; j < 1000; j++){
  // Run if data available
    if (sensorSerial.available() > 0) {
      delay(4);
  
      // Check for packet header character 0xff
      if (sensorSerial.read() == 0xff) {
        // Insert header into array
        data_buffer[0] = 0xff;
        // Read remaining 3 characters of data and insert into array
        for (int i = 1; i < 4; i++) {
          data_buffer[i] = sensorSerial.read();
        }
  
        //Compute checksum
        CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
        // If checksum is valid compose distance from data
        if (data_buffer[3] == CS) {
          distance = (data_buffer[1] << 8) + data_buffer[2];
          
          samples ++;
          sum = sum + distance;
        }
      }
    }

  } 

  mean = sum / samples;
  percentage = 100 - 100 * (mean - 200) / 2000;
  if(percentage < 0){
    percentage = 0;
  }else if(percentage > 100) {
    percentage = 100;
  }
  liters = 10000 * percentage / 100;

  char cstr[16];
  itoa(percentage, cstr, 10);
  client.publish(TOPIC_MEASUREMENTS_TANK_PERCENT,cstr);
  itoa(liters, cstr, 10);
  client.publish(TOPIC_MEASUREMENTS_TANK_LITERS,cstr);

}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  measureAndPublishTankLevel();
  delay(4000);
}
