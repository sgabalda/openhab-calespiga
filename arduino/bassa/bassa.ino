/*
  Turns on the three relays for the heater
*/
#include <UIPEthernet.h>
#include <PubSubClient.h>
#include <MemoryFree.h>

#define ARDUINO_CLIENT_ID "arduino_bassa"                     // Client ID for Arduino pub/sub
#define TOPIC_REG_SET "bassa/reg/set"             //POSSIBLE COMMANDS ARE start, stop
#define TOPIC_REG_STATUS "bassa/reg/status"       //POSSIBLE STATUS ARE off, on
#define TOPIC_CIRCULACIO_SET "bassa/circulacio/set"   //POSSIBLE_COMMANDS_ARE diposit, depuradora, stop
#define TOPIC_CIRCULACIO_STATUS "bassa/circulacio/status"   //POSSIBLE_STATUS_ARE preparing_diposit, diposit, preparing_depuradora, depuradora, preparing_off, off
#define TOPIC_AIGUAGRISA_SET "bassa/aiguagrisa/set"   //POSSIBLE_COMMANDS_ARE start, stop
#define TOPIC_AIGUAGRISA_STATUS "bassa/aiguagrisa/status"   //POSSIBLE_STATUS_ARE on, off
#define TOPIC_MEASUREMENTS_POND_LITERS "bassa/liters"
#define TOPIC_MEASUREMENTS_POND_PERCENT "bassa/percentage"
#define TOPIC_MEASUREMENTS_POND_HEIGHT "bassa/height"
#define TOPIC_MEMORY "bassa/memory"

#define MESSAGE_CIRCULACIO_OFF "off"
#define MESSAGE_CIRCULACIO_DEPURADORA "depuradora"
#define MESSAGE_CIRCULACIO_PREPARING_DEPURADORA "preparing_depuradora"
#define MESSAGE_CIRCULACIO_DIPOSIT "diposit"
#define MESSAGE_CIRCULACIO_PREPARING_DIPOSIT "preparing_diposit"
#define MESSAGE_CIRCULACIO_PREPARING_OFF "preparing_off"

#define COMMAND_START "start"
#define COMMAND_STOP "stop"
#define COMMAND_DIPOSIT "diposit"
#define COMMAND_DEPURADORA "depuradora"


#define MAX_TIME_BETWEEN_ORDERS 3*60*1000   // 3 minutes

#define RELAY_BOMBA_REG 5
#define RELAY_BOMBA_DEPURADORA 6
#define RELAY_ELECTROVALVULA 7
#define RELAY_BOMBA_AIGUAGRISA 8

// STATUS FOR BOMBA REG
#define BOMBA_REG_OFF 1  // Can transition to ON only
#define BOMBA_REG_ON 2  //can transition to OFF only

// STATUS FOR CIRCULACIO BASSA
#define CIRCULACIO_OFF 1  // Can transition to PREPARING_DIPOSIT or DEPURADORA
#define CIRCULACIO_PREPARING_DIPOSIT 2  //Can transition to OFF or PREPARING_DEPURADORA or PREPARING_DIPOSIT or PREPARING_OFF
#define CIRCULACIO_DIPOSIT 3  //can transition to PREPARING_OFF or PREPARING_DEPURADORA only
#define CIRCULACIO_PREPARING_DEPURADORA 4 //can transition to OFF or PREPARING_DIPOSIT or DEPURADORA
#define CIRCULACIO_DEPURADORA 5 //can transition to OFF or PREPARING_DIPOSIT
#define CIRCULACIO_PREPARING_OFF 6 //can transition to OFF or PREPARING_DIPOSIT or DEPURADORA

#define TRANSITION_TIME_CIRCULACIO 15 * 1000 //15 seconds

// STATUS FOR AIGUAGRISA
#define AIGUAGRISA_OFF 1  // Can transition to ON only
#define AIGUAGRISA_ON 2  //can transition to OFF only

#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Networking details
byte mac[]    = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x03 };  // Ethernet shield (W5100) MAC address
IPAddress ip(192, 168, 2, 77);                           // Ethernet shield (W5100) IP address
IPAddress server(192, 168, 2, 114);                       // MTTQ server IP address

EthernetClient ethClient;
PubSubClient client(ethClient);

//Pressure sensor calibration
double V0 = 0.52; //Volts without liquid
double V1 = 2.56; //Volts with h1 of liquid
double h1 = 1.53; // measured height at V1
double level, Vout, aux, i, result;
int previousLiters = 0;
int maxHeightCm = 153;  //max measured height

long lastOrder = 0;     //to store the last time when an order was received

int status_reg = BOMBA_REG_OFF;
int status_circulacio = CIRCULACIO_OFF;
long circulacio_last_set_depuradora = 0;
long circulacio_last_set_diposit = 0;
int status_aiguagrisa = AIGUAGRISA_OFF;

void setup() {
  //Setup default pins to off
  pinMode(RELAY_BOMBA_REG, OUTPUT);
  pinMode(RELAY_BOMBA_DEPURADORA, OUTPUT);
  pinMode(RELAY_ELECTROVALVULA, OUTPUT);
  pinMode(RELAY_BOMBA_AIGUAGRISA, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println(F("Bassa Arduino starting")); 
  
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
 
  delay(1000); // Allow hardware to stabilize 1 sec

  turnAllOff();

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

void reconnect()
{

  // Loop until reconnected
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection ... "));
    // Attempt to connect
    if (client.connect(ARDUINO_CLIENT_ID)) {
      Serial.println(F("connected"));
      // (re)subscribe
      client.subscribe(TOPIC_REG_SET);
      client.subscribe(TOPIC_CIRCULACIO_SET);
      client.subscribe(TOPIC_AIGUAGRISA_SET);
    } else {
      Serial.print(F("Connection failed, state: "));
      Serial.print(client.state());
      Serial.println(F(", retrying in 5 seconds"));
      checkMaxTime();
      delay(5000); // Wait 5 seconds before retrying
    }
    Serial.println(F("done 1st try to connect"));
  }
  Serial.println(F("Client connected"));
}

void loop() {
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  applyStates();
  measureAndReportDepthSensor();
  checkMaxTime();
  reportMemory();
  delay(2000);
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

  Serial.print(F("\n\nVoltatge: "));
  Serial.print(Vout);
  Serial.println(F(" v"));
  Serial.print(F("Nivell: "));
  Serial.print(level);
  Serial.println(F(" m"));
  
  return level;
}

double calculateVolume(double height){  //returns the volume in liters
  height = height + 0.1; //the sensor is 5cm above bottom level
  /*
  liters in function of height:
  center volume + slope volume + corner volume
  center volume = Area of center * h = 5.79 * height (center is ~ 6,5 m²)
  slope volume = 0.5 * h² / tg (alpha) = 0.5 * height * height * 0.577 (alpha = 60 deg)
  corner volume = 1/3 * h³ / (tg (alpha))² = 1/9 * height * height * height (alpha = 60 deg)

  volume = 6.5 * height + 0.5 * height * height * 0.577 + 1/9 * height * height * height
  */
  return (6.5 * height + 0.5 * height * height * 0.577 + 1/9 * height * height * height) * 1000;
}

void measureAndReportDepthSensor(){

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
  
  liters = calculateVolume(height);
  
  if(100 < abs(liters-previousLiters)){
    previousLiters = liters;
  }else{
    liters = previousLiters;
  }

  liters = (liters / 100) * 100; //round to 100L
  if(maxHeightCm < heightCm){
    maxHeightCm = heightCm;
  }
  percentage = (100.0 * liters) / calculateVolume(maxHeightCm/100.0);

  char cstr[16];
  itoa(percentage, cstr, 10);
  client.publish(TOPIC_MEASUREMENTS_POND_PERCENT,cstr);
  itoa(liters, cstr, 10);
  client.publish(TOPIC_MEASUREMENTS_POND_LITERS,cstr);
  itoa(heightCm, cstr, 10);
  client.publish(TOPIC_MEASUREMENTS_POND_HEIGHT,cstr);
  
  Serial.print(F("Percentage: "));
  Serial.println(percentage);
  Serial.print(F("liters: "));
  Serial.println(liters);
  Serial.print(F("Height: "));
  Serial.println(heightCm);

}
void applyStates(){
  if(status_circulacio == CIRCULACIO_PREPARING_DIPOSIT){
    //it was already set to transition to diposit
    if(millis() > circulacio_last_set_diposit + TRANSITION_TIME_CIRCULACIO){
      //the switch has finished the transition
      Serial.println(F("Finished transition to DIPOSIT"));
      status_circulacio = CIRCULACIO_DIPOSIT;
      client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_DIPOSIT);
    }else{
      client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DIPOSIT);
    }
  }else if(status_circulacio == CIRCULACIO_PREPARING_DEPURADORA || status_circulacio == CIRCULACIO_PREPARING_OFF ){
    //it was already set to transition to depuradora/off
    if(millis() > circulacio_last_set_depuradora + TRANSITION_TIME_CIRCULACIO){
      //the switch has finished the transition
      Serial.println(F("Finished transition to DEPURADORA/OFF"));
      if(status_circulacio == CIRCULACIO_PREPARING_DEPURADORA){
        status_circulacio = CIRCULACIO_DEPURADORA;
        client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_DEPURADORA);
      }else{
        status_circulacio = CIRCULACIO_OFF;
        client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_OFF);
      }
    }else{
      if(status_circulacio == CIRCULACIO_PREPARING_DEPURADORA){
        client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DEPURADORA);
      }else{
        client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_OFF);
      }
    }
  }
}

void checkMaxTime(){
  Serial.print(F("ms since last order: "));
  Serial.println(millis()-lastOrder);
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
  if (strcmp(topic, TOPIC_REG_SET) == 0){
    if (strcmp(message, COMMAND_START) == 0){
      turnBombaRegOn();
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnBombaRegOff();
    }else{
      Serial.println(F("-> Error, message not valid. Setting off"));
      turnBombaRegOff();
    }  
  }else if (strcmp(topic, TOPIC_AIGUAGRISA_SET) == 0){
    if (strcmp(message, COMMAND_START) == 0){
      turnAiguagrisaOn();
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnAiguagrisaOff();
    }else{
      Serial.println(F("-> Error, message not valid. Setting off"));
      turnAiguagrisaOff();
    }  
  }else if (strcmp(topic, TOPIC_CIRCULACIO_SET) == 0){
    if (strcmp(message, COMMAND_DEPURADORA) == 0){
      turnCirculacioDepuradora();
    }else if (strcmp(message, COMMAND_DIPOSIT) == 0){
      turnCirculacioDiposit();
    }else if (strcmp(message, COMMAND_STOP) == 0){
      turnCirculacioOff();
    }else{
      Serial.println(F("-> Error, message not valid. Setting off"));
      turnCirculacioOff();
    }  
  }else{
    Serial.println(F("-> Error, topic not valid"));
    turnAllOff();
  }
}

void turnAllOff(){
  
  turnAiguagrisaOff();
  turnBombaRegOff();
  turnCirculacioOff();

  Serial.println(F("All reles to Off"));
}

//transicions per aigues grises
void turnAiguagrisaOff(){
  turnRelay(RELAY_BOMBA_AIGUAGRISA, RELAY_OFF);
  client.publish(TOPIC_AIGUAGRISA_STATUS, "off");
  Serial.println(F("Aiguagrisa: off"));
}

void turnAiguagrisaOn(){
  turnRelay(RELAY_BOMBA_AIGUAGRISA, RELAY_ON);
  client.publish(TOPIC_AIGUAGRISA_STATUS, "on");
  Serial.println(F("Aiguagrisa: on"));
}

//transicions per bomba reg
void turnBombaRegOff(){
  turnRelay(RELAY_BOMBA_REG, RELAY_OFF);
  client.publish(TOPIC_REG_STATUS, "off");
  Serial.println(F("Reg: off"));
}

void turnBombaRegOn(){
  turnRelay(RELAY_BOMBA_REG, RELAY_ON);
  client.publish(TOPIC_REG_STATUS, "on");
  Serial.println(F("Reg: on"));
}

//transicions per circulacio
void turnCirculacioOff(){
  turnRelay(RELAY_ELECTROVALVULA, RELAY_OFF); //set the connection to depuradora
  turnRelay(RELAY_BOMBA_DEPURADORA, RELAY_OFF); //turn off the pump for depuradora
  if(status_circulacio == CIRCULACIO_DEPURADORA || status_circulacio == CIRCULACIO_OFF){
    status_circulacio = CIRCULACIO_OFF;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_OFF);
    Serial.println(F("Circulacio: off"));
  }else if(status_circulacio == CIRCULACIO_PREPARING_DEPURADORA || status_circulacio == CIRCULACIO_PREPARING_OFF){
    //it was already set to transition to depuradora, just change the status and wait
    status_circulacio = CIRCULACIO_PREPARING_OFF;
    Serial.println(F("Circulacio: asked for OFF, already PREP_DEP or PREP_OFF"));
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_OFF);
  }else{
    //the status is CIRCULACIO_DIPOSIT or CIRCULACIO_PREPARING_DIPOSIT
    Serial.println(F("Circulacio: asked for OFF, to PREP_OFF"));
    status_circulacio = CIRCULACIO_PREPARING_OFF;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_OFF);
    circulacio_last_set_depuradora = millis();
  }
}

void turnCirculacioDepuradora(){
  turnRelay(RELAY_ELECTROVALVULA, RELAY_OFF); //set the connection to depuradora
  if(status_circulacio == CIRCULACIO_DEPURADORA || status_circulacio == CIRCULACIO_OFF){
    turnRelay(RELAY_BOMBA_DEPURADORA, RELAY_ON); //turn on the pump for depuradora
    status_circulacio = CIRCULACIO_DEPURADORA;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_DEPURADORA);
    Serial.println(F("Circulacio: DEP"));
  }else if(status_circulacio == CIRCULACIO_PREPARING_DEPURADORA || status_circulacio == CIRCULACIO_PREPARING_OFF){
    //it was already set to transition to depuradora, ste the status and wait
    status_circulacio = CIRCULACIO_PREPARING_DEPURADORA;
    Serial.println(F("Circulacio: asked for DEP, already PREP_DEP"));
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DEPURADORA);
  }else{
    //the status is CIRCULACIO_DIPOSIT or CIRCULACIO_PREPARING_DIPOSIT
    Serial.println(F("Circulacio: asked for DEP, to PREP_DEP"));
    status_circulacio = CIRCULACIO_PREPARING_DEPURADORA;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DEPURADORA);
    circulacio_last_set_depuradora = millis();
  }
}

void turnCirculacioDiposit(){
  turnRelay(RELAY_ELECTROVALVULA, RELAY_ON); //set the connection to diposit
  turnRelay(RELAY_BOMBA_DEPURADORA, RELAY_OFF); //turn off the pump for depuradora
  if(status_circulacio == CIRCULACIO_DEPURADORA || status_circulacio == CIRCULACIO_OFF || status_circulacio == CIRCULACIO_PREPARING_DEPURADORA
    || status_circulacio == CIRCULACIO_PREPARING_OFF){
    status_circulacio = CIRCULACIO_PREPARING_DIPOSIT;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DIPOSIT);
    circulacio_last_set_diposit = millis();
    Serial.println(F("Circulacio: PREP_DIP"));
  }else if(status_circulacio == CIRCULACIO_PREPARING_DIPOSIT){
    //it was already set to transition to diposit, jsut wait
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_PREPARING_DIPOSIT);
    Serial.println(F("Circulacio: asked for DIP, already PREP_DIP"));
  }else{
    //the status is CIRCULACIO_DIPOSIT
    Serial.println(F("Circulacio: asked for DIP, to DIP"));
    status_circulacio = CIRCULACIO_DIPOSIT;
    client.publish(TOPIC_CIRCULACIO_STATUS, MESSAGE_CIRCULACIO_DIPOSIT);
  }
}

void turnRelay(int relay, int status) {
  digitalWrite(relay, status);
  delay(100);
}
