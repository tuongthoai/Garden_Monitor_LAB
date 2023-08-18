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
//define mqtt
const char* mqttServer = "test.mosquitto.org";
int port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);
//define wifi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
LiquidCrystal_I2C LCD(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
DHTesp dhtSensor;

struct SensorsData {
  float temperature;
  float humidity;
  uint16_t moiser;
  uint16_t rain;
  uint8_t light;

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

  DeviceStatus() {
    waterPump = 0;
    microWaterPump = 0;
    heatLight = 0;
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Serial.print("Connecting to Wifi");
  wifiConnect();

  //connect mqtt
  // client.setServer(mqttServer,port);
  // mqttReconnect();
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

void loop() {
  curTime = millis();
  if (curTime - lastTime > 1000) {
    //read Sensor data
    readSensorData();

    //compare value if has different
    isDiff = curData != lastData;
    
    if (isDiff) {
      //handle condition of sensor here
      Serial.printf("%.1f %.1f\n", curData.humidity, curData.temperature);
    }
  }

  //handle status of output devices
  //roof motor
  if( myStepper.distanceToGo() != 0 ) {
    myStepper.run();
  }
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
    curData.moiser = analogRead(P_DoAmDat);
    curData.light = digitalRead(P_AnhSang);
    curData.rain = digitalRead(P_Mua);
}

void openRainDefender() {

}

void closeRainDefender() {

}

void writeLCD(uint8_t page){

}
/*
  // if (myStepper.distanceToGo() == 0) 
	// 	myStepper.moveTo(-myStepper.currentPosition());

	// // Move the motor one step
	// myStepper.run();
  // myStepper.run();
*/