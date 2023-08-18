#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHTesp.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include "PubSubClient.h"

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
void writeLCD(uint8_t page);

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Connecting to Wifi");
  wifiConnect();

  //connect mqtt
  // client.setServer(mqttServer,port);
  // client.setCallback(mqtt_callback);

  LCD.init();
  LCD.backlight();
  LCD.setCursor(0, 0);
  LCD.print("Hello world!");
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
}

void CN_TuoiNuoc();
void CN_PhunSuong();
void CN_DenSuoi();
void CN_ManChe();

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
    // publishToConsumer();
  }

  if (newStatus != curStatus) {
    // Xu ly cac trang thai cua thiet bi output theo tung chuc nang
    for (int i = 0; i < 4; ++i) {
      functionalPointers[i]();
    }

    // Tien hanh thay doi trang thai cua cac thiet bi output theo trang thai da tinh toan
    for(int i = 0; i < 4; ++i) {
      actionPointers[i]();
    }

    // Cap nhat lai trang thai hien tai la trang thai da tinh toan
    curStatus = newStatus;
  }

  //handle status of output devices
  //roof motor
  if( myStepper.distanceToGo() != 0 ) {
    myStepper.run();
  }
}

void mqtt_callback(char* topic, byte* payload, uint32_t len){
  // Serial.print("Message received on topic: ");
  // Serial.println(topic);

  // Serial.print("Payload: ");
  // for (int i = 0; i < len; i++) {
  //   Serial.print((char)payload[i]);
  // }
  // Serial.println();

  //Processing topic here

  //Processing msg from node_red here

  Serial.print(topic);
  String strMessage;
  for(int  i = 0; i<len; i++){
    strMessage += (char)payload[i];
  }
  Serial.println(strMessage);
}

void publishToConsumer() //publish msg to Consumer
{
  int temperature = curData.temperature;
  int humidity = curData.humidity;
  int moiser = curData.moiser;
  int light = curData.light;
  int rain = curData.rain;

  char buffer[20];
  sprintf(buffer, "%d", temperature);
  client.publish("21127174/temperature",buffer);

  sprintf(buffer, "%d", humidity);
  client.publish("21127174/humidity",buffer);

  sprintf(buffer, "%d", moiser);
  client.publish("21127174/moiser",buffer);

  sprintf(buffer, "%d", rain);
  client.publish("21127174/rain",buffer);

  sprintf(buffer, "%d", curStatus.heatLight);
  client.publish("21127174/heatLight",buffer);

  sprintf(buffer, "%d", curStatus.waterPump);
  client.publish("21127174/waterPump",buffer);

  sprintf(buffer, "%d", curStatus.microWaterPump);
  client.publish("21127174/microWaterPump",buffer);
}

//from mqtt to ESP32
void mqttReconnect(){
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    if(client.connect("21127174")){
      Serial.println("connected");
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
    curData.light = digitalRead(P_AnhSang);
    curData.rain = digitalRead(P_Mua);
}

void openRainDefender() {
  myStepper.moveTo(FULL_OPEN);
}

void closeRainDefender() {
  myStepper.moveTo(FULL_CLOSE);
}

void writeLCD(uint8_t page){
  
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
  if(newStatus.microWaterPump == 0) {
    if ((curData.rain == 0) && (curData.moiser < 55)){
      if ((curData.temperature > 30) && (curData.light == 1)){
        newStatus.waterPump == 1;
      }
    }
    else if ((curData.rain == 0) && (curData.moiser < 50)){
      if ((curData.temperature > 25) && (curData.light == 0)){
        newStatus.waterPump == 1;
      }
    }
    else if((curData.rain == 0) && (curData.moiser < 50)){
      newStatus.waterPump == 1;
    }
    else newStatus.waterPump == 0;
  }
}

void CN_PhunSuong() {
  if(newStatus.waterPump == 0) {
    if ((curData.humidity < 70) && ((curData.rain == 0))){
      if ((curData.temperature > 30) && (curData.light == 1)){
        newStatus.microWaterPump = 1;
      }
      else newStatus.microWaterPump = 0;
    }
    else if ((curData.humidity < 65) && ((curData.rain == 0))) {
      if ((curData.temperature > 25) && (curData.light == 0)){
        newStatus.microWaterPump = 1;
      }
      else newStatus.microWaterPump = 0;
    }
    else if (curData.rain == 0){
      if (curData.humidity < 65){
        newStatus.microWaterPump = 1;
      }
      else newStatus.microWaterPump = 0;
    }
    else newStatus.microWaterPump = 0;
  }
}

void CN_DenSuoi() {
  if ((curData.temperature < 20) && (curData.light == 1)){
    newStatus.heatLight = 1;
  }
  else if((curData.temperature < 16) && (curData.light == 0)){
    newStatus.heatLight = 1;
  }
  else newStatus.heatLight = 0;
}

void CN_ManChe() {
  if(curData.light == 0){
    newStatus.roofTop = 0;
  }
  else if((curData.light == 1) && (curData.temperature > 30)){
    newStatus.roofTop = 0;
  }
  else if ((curData.rain == 1) && (curData.moiser > 70)){
    newStatus.roofTop = 0;
  }
  else if((curData.light == 1) && (curData.temperature < 20)){
    newStatus.roofTop = 1;
  }
  else if ((curData.rain == 1) && (curData.moiser < 50)){
    newStatus.roofTop = 1;
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
    digitalWrite(P_BomNuoc, (newStatus.microWaterPump > 0));
  }
}
void actionDenSuoi() {
  if (curStatus.heatLight != newStatus.heatLight) {
    digitalWrite(P_BomNuoc, (newStatus.heatLight > 0));
  }
}
void actionManChe() {
  if (curStatus.roofTop) {
    if (newStatus.roofTop) {
      openRainDefender();
    } else {
      closeRainDefender();
    }
  }
}