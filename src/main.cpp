#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHTesp.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "ThingSpeak.h"

#define Max_Humidity 80
#define Min_Humidity 60
#define Max_Temperature_Day 30
#define Min_Temperature_Day 20
#define Max_Temperature_Night 25
#define Min_Temperature_Night 15
#define Max_Moiser 70
#define Min_Moiser 55

#define P_BomNuoc 27
#define P_PhunSuong 26
#define P_NhietDoKK 25
#define P_DoAmDat 33
#define P_DenSuoi 32
#define P_AnhSang 35
#define P_Mua 34

#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4

#define dirPin 15
#define stepPin 2
#define READ_INTERVAL 500
#define FULL_OPEN 600
#define FULL_CLOSE -600
#define EPSILON 1e-1

// Define motor interface type
#define motorInterfaceType 1

//define wifi
const char* mqttServer = "test.mosquitto.org";
const char* ssid = "Wokwi-GUEST";
const char* password = "";

//define mqtt
const uint16_t port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

//ThingSpeak setting
const unsigned long channelId = 2245470;
const char* writeAPI = "CC4W6GQVHGJDBM4F";
const char* readAPI = "5HUF2J6V10OP77ZX";
WiFiClient TS_client;

//IFTTT
const char* host = "maker.ifttt.com";
const char* request = "/trigger/values_changed/with/key/XEG0gX_3bpvefCvAQC6VG";
int port_IFTTT = 80;

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
LiquidCrystal_I2C LCD(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
DHTesp dhtSensor;


struct SensorsData {
  float temperature;
  float humidity;
  uint16_t moiser;
  uint16_t rain;
  uint16_t light;

  SensorsData(){
    temperature = 0;
    humidity = 0;
    moiser = 0;
    light = 0;
    rain = 0;
  }

  friend bool operator==(SensorsData& a, SensorsData& b);
  friend bool operator!=(SensorsData& a, SensorsData& b);
};

struct DeviceStatus {
  uint8_t waterPump;
  uint8_t microWaterPump;
  uint8_t heatLight;
  uint8_t roofTop;

  DeviceStatus() {
    waterPump = 0;
    microWaterPump = 0;
    heatLight = 0;
    roofTop = 0;
  }

  friend bool operator==(DeviceStatus& a, DeviceStatus& b);
  friend bool operator!=(DeviceStatus& a, DeviceStatus& b);
};

// setup wifi
void wifiConnect();
//from mqtt to ESP32
void mqttReconnect();

// MQTT functionality
void publishToConsumer(); //publish msg to Consumer
void mqtt_callback(char* topic, byte* payload, uint32_t len);
bool StringEqual(const char* a, const char* b);
void receiveFromUI(char* topic);

//global variables
uint64_t curTime = 0, lastTime = 0;
SensorsData lastData,curData;
DeviceStatus curStatus, newStatus;
bool isDiff;

// for open and closing 
void openRainDefender();  
void closeRainDefender();
bool isClose(float& a, float& b);
void readSensorData();
void writeLCD();

// functional handling
void CN_TuoiNuoc();
void CN_PhunSuong();
void CN_DenSuoi();
void CN_ManChe();

void actionBomNuoc();
void actionPhunSuong();
void actionDenSuoi();
void actionManChe();

void (*functionalPointers[])() = {CN_TuoiNuoc, CN_PhunSuong, CN_DenSuoi, CN_ManChe};
void (*actionPointers[])() = {actionBomNuoc, actionDenSuoi, actionManChe, actionPhunSuong};

//ThingSpeak
void sendRequest();
void sendMessage();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Connecting to Wifi");
  wifiConnect();

  //connect mqtt
  client.setServer(mqttServer,port);
  client.setCallback(mqtt_callback);

  LCD.init();
  LCD.backlight();
  dhtSensor.setup(P_NhietDoKK, DHTesp::DHT22);

  pinMode(P_DoAmDat, INPUT);
  pinMode(P_Mua, INPUT);
  pinMode(P_AnhSang, INPUT);
  pinMode(P_BomNuoc, OUTPUT);
  pinMode(P_PhunSuong, OUTPUT);
  pinMode(P_DenSuoi, OUTPUT);

  myStepper.setMaxSpeed(1000);
	myStepper.setAcceleration(100);
	myStepper.setSpeed(100);

  //ThingSpeak
  ThingSpeak.begin(TS_client);
}

void loop() {
  curTime = millis();

  // mqttReconnect();
  // client.loop();

  if (curTime - lastTime > 500) {
    //read Sensor data
    readSensorData();

    //compare value if has different
    isDiff = curData != lastData;
    lastTime = curTime;
  }

  if (isDiff) {
    //handle condition of sensor here
    // Serial.printf("%.1f %.1f %d %d %d\n", curData.humidity, curData.temperature, curData.moiser, curData.light, curData.rain);
    lastData = curData;
    // Xu ly cac trang thai cua thiet bi output theo tung chuc nang
    for (int i = 0; i < 4; ++i) {
      functionalPointers[i]();
    }
    writeLCD();
    publishToConsumer();

    if (newStatus != curStatus) {
      // Xu ly cac trang thai cua thiet bi output theo tung chuc nang

      for (int i = 0; i < 4; ++i) {
        actionPointers[i]();
      }
      // publishToConsumer();
      Serial.printf("OLD MC %d PS %d BN %d DS %d\n", curStatus.roofTop, curStatus.microWaterPump, curStatus.waterPump, curStatus.heatLight);
      Serial.printf("NEW MC %d PS %d BN %d DS %d\n", newStatus.roofTop, newStatus.microWaterPump, newStatus.waterPump, newStatus.heatLight);
      curStatus = newStatus;
    }
    Serial.printf("Data AmKK %.1f T* %.1f AmDat %d Mua %d Sang %d\n", curData.humidity, curData.temperature, curData.moiser, curData.rain, curData.light);
  }

  //handle status of output devices
  //roof motor
  if( myStepper.distanceToGo() != 0 ) {
    myStepper.run();
  }
  
  //ThingSpeak
  ThingSpeak.setField(1, curData.temperature);
  ThingSpeak.setField(2, curData.moiser);
  ThingSpeak.setField(3, curData.humidity);
  int ret = ThingSpeak.writeFields(channelId, writeAPI);
  if(ret == 200){
    Serial.println("Successful");
  }
  else{
    Serial.println("Error");
  }
  float temp = ThingSpeak.readLongField(channelId, 1, readAPI);
  ret = ThingSpeak.getLastReadStatus();
  if (ret == 200){
    char buffer[20];
    sprintf(buffer, "%d", int(temp));
    client.publish("cloud",buffer);
  }
  else{
    Serial.println("Unable to read channel");
  }
  sendMessage();
  // delay(1000);
}

void mqtt_callback(char* topic, byte* payload, uint32_t len){
  Serial.print(topic);
  String strMessage;
  for(int  i = 0; i<len; i++){
    strMessage += (char)payload[i];
  }
  Serial.println(strMessage);
  receiveFromUI(topic)
}

void receiveFromUI(char* topic){
  if (StringEqual(topic, "microWaterPump_subscribe"))
  {
    if (curStatus.microWaterPump == 0) {
      newStatus.microWaterPump = 1;
    } 
    else {
      newStatus.microWaterPump = 0;
    }
  }
  else if (StringEqual(topic, "waterPump_subscribe"))
  {
    if (curStatus.waterPump == 0) {
      newStatus.waterPump = 1;
    } 
    else {
      newStatus.waterPump = 0;
    }
  }
  else if (StringEqual(topic, "roofTop_subscribe"))
  {
    if (curStatus.roofTop == 0) {
      newStatus.roofTop = 1;
    } 
    else {
      newStatus.roofTop = 0;
    }
  }
  else if (StringEqual(topic, "heatLight_subscribe"))
  {
    if (curStatus.heatLight == 0) {
      newStatus.heatLight = 1;
    } 
    else {
      newStatus.heatLight = 0;
    }
  }
}


void publishToConsumer() //publish msg to Consumer
{
  char buffer[20];
  sprintf(buffer, "%d", int(curData.temperature));
  client.publish("temperature",buffer);

  sprintf(buffer, "%d", int(curData.humidity));
  client.publish("humidity",buffer);

  sprintf(buffer, "%d", curData.moiser);
  client.publish("moiser",buffer);

  sprintf(buffer, "%d", curData.rain);
  client.publish("rain",buffer);

  sprintf(buffer, "%d", curStatus.heatLight);
  client.publish("heatLight",buffer);

  sprintf(buffer, "%d", curStatus.waterPump);
  client.publish("waterPump",buffer);

  sprintf(buffer, "%d", curStatus.microWaterPump);
  client.publish("microWaterPump",buffer);

  sprintf(buffer, "%d", curStatus.roofTop);
  client.publish("roofTop",buffer);
}

//from mqtt to ESP32
void mqttReconnect(){
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    if(client.connect("GARDENROSE")){
      Serial.println("connected");
      client.subscribe("microWaterPump_subscribe");
      client.subscribe("roofTop_subscribe");
      client.subscribe("microWaterPump_subscribe");
      client.subscribe("microWaterPump_subscribe");
    }
    else{
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// setup wifi
void wifiConnect(){
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
}

bool isClose(float& a, float& b){
  return std::abs(a - b) < EPSILON;
}

void readSensorData() {
    TempAndHumidity tmp = dhtSensor.getTempAndHumidity();
    curData.humidity = tmp.humidity;
    curData.temperature = tmp.temperature;
    curData.moiser = map(analogRead(P_DoAmDat), 0, 1023, 0, 25);
    curData.light = 1 - digitalRead(P_AnhSang);
    curData.rain = digitalRead(P_Mua);
}

void openRainDefender() {
  myStepper.moveTo(FULL_OPEN);
}

void closeRainDefender() {
  myStepper.moveTo(FULL_CLOSE);
}

void writeLCD() {
  LCD.setCursor(0, 0);
  LCD.print("T\": ");
  LCD.print(curData.temperature, 1);
  LCD.print(" *C");
  LCD.setCursor(0, 1);
  LCD.print("Humid: ");
  LCD.print(curData.humidity, 1);
  LCD.print("%");
  LCD.setCursor(0, 2);
  LCD.print("Moiser: ");
  LCD.print(curData.moiser);
  LCD.print("%");
}

bool operator==(SensorsData& a, SensorsData& b){
  return (a.rain == b.rain) && (a.moiser == b.moiser) && (a.light == b.light) && isClose(a.temperature, b.temperature) && isClose(a.humidity, b.humidity);
}
bool operator!=(SensorsData& a, SensorsData& b){
  return !(a == b);
}

bool operator==(DeviceStatus& a, DeviceStatus& b) {
  return (a.waterPump == b.waterPump) && (a.heatLight == b.heatLight) && (a.microWaterPump == b.microWaterPump);
}
bool operator!=(DeviceStatus& a, DeviceStatus& b) {
 return !(a == b);
}

void CN_TuoiNuoc() {
  if ((curData.rain == 0) && (curData.moiser < Min_Moiser)){
    newStatus.waterPump == 1;
  }
  else newStatus.waterPump == 0;
}

void CN_PhunSuong() {
  if ((curData.temperature > Max_Temperature_Day)){
    newStatus.microWaterPump = 1;
    return;
  }
  else if ((curData.humidity < Min_Humidity) && ((curData.rain == 0))) {
    newStatus.microWaterPump = 1; 
    return;
  }
  else newStatus.microWaterPump = 0;
}

void CN_DenSuoi() {
  if ((curData.temperature < Min_Temperature_Day) && (curData.light == 1)){
    newStatus.heatLight = 1;
  }
  else if((curData.temperature < Min_Temperature_Night) && (curData.light == 0)){
    newStatus.heatLight = 1;
  }
  else newStatus.heatLight = 0;
}

void CN_ManChe() {
  if(curData.light == 0){
    newStatus.roofTop = 0;
    return;
  }
  else if ((curData.rain == 1) && (curData.light == 1) && (curData.temperature < Max_Temperature_Day) && (curData.temperature > Min_Temperature_Day) && (curData.moiser < Max_Moiser)){
    newStatus.roofTop = 1;
    return;
  }
  else if ((curData.rain == 0) && (curData.light == 1) && (curData.temperature < Min_Temperature_Day)){
    newStatus.roofTop = 1;
    return;
  }
  else if ((curData.rain == 1) && (curData.moiser < Min_Moiser)){
    newStatus.roofTop = 1;
    return;
  }
  else newStatus.roofTop = 0;
}

void actionBomNuoc() {
  if (curStatus.waterPump != newStatus.waterPump) {
    digitalWrite(P_BomNuoc, (newStatus.waterPump > 0));
  }
}
void actionPhunSuong() {
  if (curStatus.microWaterPump != newStatus.microWaterPump) {
    digitalWrite(P_PhunSuong, (newStatus.microWaterPump > 0));
  }
}
void actionDenSuoi() {
  if (curStatus.heatLight != newStatus.heatLight) {
    digitalWrite(P_DenSuoi, (newStatus.heatLight > 0));
  }
}
void actionManChe() {
  // if (curStatus.roofTop) {
    if (newStatus.roofTop) {
      openRainDefender();
    } else {
      closeRainDefender();
    }
  // }
}

void sendRequest(){
  Serial.println("Connectinng to ");
  Serial.println(host);
  Serial.println(": ");
  Serial.println(port_IFTTT);

  WiFiClient client;
  while (!client.connect(host, port_IFTTT)){
    Serial.println("connection fail");
    delay(1000);
  }

  client.print(("GET ") + String(request) + " HTTP/1.1\r\n" + 
                "Host: " + host + "\r\n" +
                "Connection: close\r\n\r\n");
  delay(500);

  while(client.available()){
    String line = client.readStringUntil('\R');
    Serial.println(line);
  }
  Serial.println();
}

void sendMessage(){
  if((curData.humidity > Max_Humidity) || (curData.temperature > Max_Temperature_Day)){
    sendRequest();
  }
}

bool StringEqual(const char* a, const char* b) // bruh moment :>
{
    while (*a != '\0' && *b != '\0') 
    {
        if (*a != *b)
            return false;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}