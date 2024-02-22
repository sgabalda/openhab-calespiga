#include <WiFi.h>
#include <PubSubClient.h>

#define ARDUINO_CLIENT_ID "esp32_estufa_infrarrojos"                     // Client ID for Arduino pub/sub
#define SUB_ESTUFA_POT "estufa1/set"
#define PUB_ESTUFA_POT "estufa1/status"
#define MAX_TIME_BETWEEN_ORDERS 3*60*1000   // 3 minutes, if the MQTT goes nuts, turn off the heater just in case

// Replace the next variables with your SSID/Password combination
const char* ssid = "Ca l'espiga";
const char* password = "casadepalla";
const char* mqtt_server = "192.168.2.114";

byte relayON = HIGH;
byte relayOFF = LOW;

long lastOrder = 0;     //to store the last time when an order was received

WiFiClient espClient;
PubSubClient client(espClient);

int RELAY_1 = 32;
int RELAY_2 = 33;

byte relay_1_status = relayOFF;
byte relay_2_status = relayOFF;

void setup() {
  //Setup default pins to off
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);

  Serial.begin(9600);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  Serial.println(message);

  if (strcmp(message, "0") == 0){
      lastOrder=millis();
      estufaOff();
  }else if (strcmp(message, "600") == 0){
      lastOrder=millis();
      estufa600W();
  }else if (strcmp(message, "1200") == 0){
      lastOrder=millis();
      estufa1200W();
  }else{
      Serial.println("Error, message not valid:");
      Serial.println(message);
      estufaOff();
  }


  // Feel free to add more if statements to control more GPIOs with MQTT

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(SUB_ESTUFA_POT);
      delay(1000);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void checkMaxTime(){
  Serial.print("Time since last order is: ");
  Serial.print(millis()-lastOrder);
  Serial.println("ms");
  if(millis()-lastOrder > MAX_TIME_BETWEEN_ORDERS){
    Serial.print("Too much time between orders, turning off estufa:");
    Serial.print(millis()-lastOrder);
    Serial.println("ms");
    estufaOff();
    lastOrder=millis();
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  checkMaxTime();
  applyRelayStatus();
  delay(3000);
  Serial.print("Free memory is: ");
  Serial.println(ESP.getFreeHeap());
}

void estufaOff(){
  //turn off in the opposite order, thus relay_1 does not receive a lot of current
  relay_1_status = relayOFF;
  relay_2_status = relayOFF;
}

void estufa600W(){
  relay_1_status = relayOFF;
  relay_2_status = relayON;
}

void estufa1200W(){
  relay_1_status = relayON;
  relay_2_status = relayON;
}

void applyRelayStatus(){
  turnRelay(RELAY_1, relay_1_status);
  turnRelay(RELAY_2, relay_2_status);
  if(relay_1_status == relayOFF && relay_2_status == relayOFF){
    client.publish(PUB_ESTUFA_POT, "0");
  }else if(relay_1_status == relayOFF || relay_2_status == relayOFF){
    client.publish(PUB_ESTUFA_POT, "600");
  }else{
    client.publish(PUB_ESTUFA_POT, "1200");
  } 
}

void turnRelay(int relay, int status) {
  Serial.print("Relay ");
  Serial.print(relay);
  Serial.print(" set to ");
  Serial.println(status);
  digitalWrite(relay, status);
  delay(1000);
}