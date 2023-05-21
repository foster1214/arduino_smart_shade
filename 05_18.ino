#include <PPPOS.h>
#include <PPPOSClient.h>
#include <PubSubClient.h>

#include "TYPE1SC.h"

#define SERIAL_BR 115200
#define GSM_SERIAL 1
#define GSM_RX 16
#define GSM_TX 17
#define GSM_BR 115200

#define PWR_PIN 5
#define RST_PIN 18
#define WAKEUP_PIN 19

#define DebugSerial Serial
#define M1Serial Serial2 // ESP32

char *server = "example.com";
char *ppp_user = "codezoo";
char *ppp_pass = "codezoo";
String APN = "simplio.apn";
char *SUB_TOPIC = "type1sc/100/control";
char *PUB_TOPIC = "type1sc/100/status";
char *PUB_TOPIC_FORCE = "type1sc/100/force_status";
char *PUB_TOPIC_LIGHT = "type1sc/100/sensors/light";
char *PUB_TOPIC_TEMP = "type1sc/100/sensors/temp";
char *PUB_TOPIC_WIND = "type1sc/100/sensors/wind";

#define MQTT_SERVER "broker.hivemq.com"
String buffer = "";
char *data = (char *)malloc(1024);

PPPOSClient ppposClient;
PubSubClient client(ppposClient);
bool atMode = true;

TYPE1SC TYPE1SC(M1Serial, DebugSerial, PWR_PIN, RST_PIN, WAKEUP_PIN);

bool openStatus = false;     //어닝 펼쳐짐 여부
bool ledStatus = false;      //LED 켜짐 여부
bool weatherStatus = false;  //폭염, 강풍 등 여부
bool forceOpen = false;       //어닝 강제 펼침 여부
bool forceOn = false;         //LED 강제 켜짐 여부

//온도 센서
int lm35Pin = 15;
int tempRaw = 0;
float Voltage = 0;
float tempC = 0;
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0

//조도 센서
int Light_Signal = 4;
int light = 0;

//서보 모터
int servoPin = 23;

//setting PWM properties
const int servoChannel = 0;
const int freq = 50;
const int resolution = 16;

int deg, duty;

//풍속 센서
int anemometerPin = 2;
float windVal = 0;
float windVal_timer = 0;

//LED
int LED_R = 32;
int LED_G = 33;
int LED_B = 34;
int LED_Y = 25;

unsigned long time_previous, time_current, wind_timer;

char sensVal[20];
char tempVal[8];
char lightVal[4];
char windValue[8];

void callback(char *topic, byte *payload, unsigned int length) {
  // Allocate the correct amount of memory for the payload copy
  byte *p = (byte *)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p, payload, length);

  //  DebugSerial.print("Message arrived [");
  //  DebugSerial.print(topic);
  //  DebugSerial.print("] ");
  //  for (int i = 0; i < length; i++) {
  //    DebugSerial.print((char)payload[i]);
  //  }
  //  DebugSerial.println();

  if (strstr((char *)p, "on")) {
    componentControl("on");

  } else if (strstr((char *)p, "off")) {
    componentControl("off");

  } else if (strstr((char *)p, "open")){
    componentControl("open");

  } else if (strstr((char *)p, "close")){
    componentControl("close");
    
  } else if (strstr((char *)p, "esc")){
    componentControl("esc");

  } else if (strstr((char *)p, "dis")) {
    PPPOS_stop();
    atMode = true;
    if (TYPE1SC.setAT() == 0) {
      DebugSerial.println("Command Mode");
    } else {
      atMode = false;
    }
  }
  //  client.publish(PUB_TOPIC, p, length);
  free(p);
}

bool startPPPOS() {
  PPPOS_start();
  unsigned long _tg = millis();
  while (!PPPOS_isConnected()) {
    DebugSerial.println("ppp Ready...");
    if (millis() > (_tg + 30000)) {
      PPPOS_stop();
      return false;
    }
    delay(3000);
  }

  DebugSerial.println("PPPOS Started");
  return true;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    DebugSerial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("catm1Client")) {
      DebugSerial.println("connected");
      client.subscribe(SUB_TOPIC);
      // Once connected, publish an announcement...
      client.publish(PUB_TOPIC, "Device Ready");
      // ... and resubscribe
    } else {
      DebugSerial.print("failed, rc=");
      DebugSerial.print(client.state());
      DebugSerial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  /* CATM1 Modem PowerUp sequence */
  pinMode(PWR_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(WAKEUP_PIN, OUTPUT);

  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(WAKEUP_PIN, HIGH);
  digitalWrite(RST_PIN, LOW);
  delay(100);
  digitalWrite(RST_PIN, HIGH);
  delay(2000);
  /********************************/
  // put your setup code here, to run once:
  M1Serial.begin(SERIAL_BR);
  DebugSerial.begin(SERIAL_BR);

  DebugSerial.println("TYPE1SC Module Start!!!");

  /* TYPE1SC Module Initialization */
  if (TYPE1SC.init()) {
    DebugSerial.println("TYPE1SC Module Error!!!");
  }

  /* Network Regsistraiton Check */
  while (TYPE1SC.canConnect() != 0) {
    DebugSerial.println("Network not Ready !!!");
    delay(2000);
  }

  /* Get Time (GMT, (+36/4) ==> Korea +9hour) */
  char szTime[32];
  if (TYPE1SC.getCCLK(szTime, sizeof(szTime)) == 0) {
    DebugSerial.print("Time : ");
    DebugSerial.println(szTime);
  }
  delay(1000);

  int rssi, rsrp, rsrq, sinr;
  for (int i = 0; i < 3; i++) {
    /* Get RSSI */
    if (TYPE1SC.getRSSI(&rssi) == 0) {
      DebugSerial.println("Get RSSI Data");
    }
    delay(1000);

    /* Get RSRP */
    if (TYPE1SC.getRSRP(&rsrp) == 0) {
      DebugSerial.println("Get RSRP Data");
    }
    delay(1000);

    /* Get RSRQ */
    if (TYPE1SC.getRSRQ(&rsrq) == 0) {
      DebugSerial.println("Get RSRQ Data");
    }
    delay(1000);

    /* Get SINR */
    if (TYPE1SC.getSINR(&sinr) == 0) {
      DebugSerial.println("Get SINR Data");
    }
    delay(1000);
    int count = 3 - (i + 1);
    DebugSerial.print(count);
    DebugSerial.println(" left..");
  }

  if (TYPE1SC.setPPP() == 0) {
    DebugSerial.println("PPP mode change");
    atMode = false;
  }

  String RF_STATUS = "[RF Status] RSSI: " + String(rssi) +
                     " RSRP:" + String(rsrp) + " RSRQ:" + String(rsrq) +
                     " SINR:" + String(sinr);
  DebugSerial.println(RF_STATUS);
  DebugSerial.println("TYPE1SC Module Ready!!!");

  /* PPPOS Setup */
  PPPOS_init(GSM_TX, GSM_RX, GSM_BR, GSM_SERIAL, ppp_user, ppp_pass);
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);
  DebugSerial.println("Starting PPPOS...");

  if (startPPPOS()) {
    DebugSerial.println("Starting PPPOS... OK");
  } else {
    DebugSerial.println("Starting PPPOS... Failed");
  }

  //시작시간
  time_previous = millis(); 
  wind_timer = millis();

  Serial.begin(115200);

  //LED 핀 설정
  //온도 표시
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  //보안등
  pinMode(LED_Y, OUTPUT);

  //SERVO 모터 설정
  ledcSetup(servoChannel, freq, resolution);
  ledcAttachPin(servoPin, servoChannel);

}

void loop() {
  if (PPPOS_isConnected() && !atMode) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  gatherSensorVal(); 
  gatherWindSensorVal();

  windVal_timer = windVal;

  componentCondition(light < 30);
  
  time_current = millis();

  if(time_current - wind_timer >= 2000){
    wind_timer = time_current;
    gatherWindSensorVal();
    if (windVal_timer > 7 && windVal > 7 && openStatus == true){
      componentControl("close");
      client.publish(PUB_TOPIC_FORCE, "Strong Wind Dectected");
    }
  }

  if(time_current - time_previous >= 10000){
    time_previous = time_current;
    sendVal();
    printData();
  }
}

//센서 값 저장
void gatherSensorVal(){
  //조도 센서값 저장
  light = map(analogRead(Light_Signal), 0, 4096, 0, 100);

  tempRaw = analogRead(lm35Pin);
  Voltage = tempRaw * (ADC_VREF_mV / ADC_RESOLUTION);
  tempC = Voltage / 10;

}

void gatherWindSensorVal(){
  //풍속 센서값 저장
  float windSensor = analogRead(anemometerPin);
  float windCal = windSensor / 4096.0 * 5.0;
  windVal = windCal * 6;
}

//조건에 맞춰 서보 모터, 조명 작동
void componentCondition(bool condition){
  //밤
  if (condition == true) {

    if(ledStatus == false && forceOn == false){
      ledControl(ledStatus, forceOn);
      ledStatus = true;
    }

    if (tempC >= 0 & tempC < 25) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);

    } else if (tempC >= 26 & tempC < 30) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);

    } else if (tempC >= 31) {

      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
    }

    delay(2500);
  }
  //낮
  else if (condition == false) {

    if(ledStatus == true && forceOn == false){
      ledControl(ledStatus, forceOn);
      ledStatus = false;
    }

    if (tempC >= 0 & tempC < 25) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);

      if (openStatus == true && forceOpen == false) {
        //닫힘
        motorControl(openStatus);
        delay(1000);
        openStatus = false;
      }

      delay(1000);
    } else if (tempC >= 26 & tempC < 30) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);

      if(openStatus == false && forceOpen == false){
        //열림
        motorControl(openStatus);
        openStatus = true;
        delay(1000);
      }

    } else if (tempC >= 31) {

      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
      
      if(openStatus == false && forceOpen == false){
        //열림
        motorControl(openStatus);
        openStatus = true;
        delay(1000);
      }

    }

    delay(2500);
  }
}

//시리얼 통신을 통해 특정 단어 입력 시 컴포넌트 조작
void componentControl(String component) {

  //어닝, LED 강제 켜짐
  //esc 전송 시 탈출
  if(component == "esc"){
    if(forceOpen == true){
      forceOpen = false;  
    }

    if(forceOn == true){
      forceOn = false; 
    }
  }

  //open 입력 시 어닝 펼침
  //이후 yes 입력 시 탈출
  if (component == "open") {

    if (openStatus == true) {
      Serial.println("Awning is already Opened");
    } else if (openStatus == false) {
      Serial.println("Awning Open");
      motorControl(openStatus);
      openStatus = true;
      forceOpen = true;   
      client.publish(PUB_TOPIC_FORCE, "Awning Force Open");  
    }

    //close 입력 시 어닝 접음
    //이후 yes 입력 시 탈출
  } else if (component == "close") {

    if (openStatus == false) {
      Serial.println("Awning is already Closed");
    } else if (openStatus == true) {
      Serial.println("Awning Close");
      motorControl(openStatus);
      openStatus = false;
      forceOpen = true;
      client.publish(PUB_TOPIC_FORCE, "Awning Force Close");  
    }

  }

  //on 입력 시 LED 작동
  //이후 yes 입력 시 탈출
  if (component == "on") {

    if (ledStatus == true) {
      Serial.println("LED is already ON");
    } else if (ledStatus == false) {
      Serial.println("LED ON");
      ledControl(ledStatus, forceOn);
      ledStatus = true;
      forceOn = true;
      client.publish(PUB_TOPIC_FORCE, "LED Force On");  
    }

    //off 입력 시 LED 꺼짐
    //이후 yes 입력 시 탈출
  } else if (component == "off") {

    if (ledStatus == false) {
      Serial.println("LED is already OFF");
    } else if (ledStatus == true) {
      Serial.println("LED OFF");
      ledControl(ledStatus, forceOn);
      ledStatus = false;
      forceOn = true;
      client.publish(PUB_TOPIC_FORCE, "LED Force Off");  
    }
  }
}


// 데이터 수신 시 모터, LED 제어
// 시리얼 모니터에 1 전송 시 어닝 테스트
// 시리얼 모니터에 2 전송 시 LED 테스트
void componentTest() {
  char ch = Serial.read();
  if (ch == 49) {
    Serial.println("MOTOR TEST ON");
    if(openStatus == true){
      motorControl(openStatus);
      openStatus = false;
      motorControl(openStatus);
      openStatus = true;
      delay(2500);
    } else {
      motorControl(openStatus);
      openStatus = true;
      motorControl(openStatus);
      openStatus = false;
      delay(2500);
    }
    Serial.println("MOTOR TEST DONE");
  } else if (ch == 50) {
    Serial.println("LED TEST ON");
    ledControl(ledStatus, forceOn);
    delay(2500);
    ledControl(ledStatus, forceOn);
    delay(2500);
    Serial.println("LED TEST DONE");
  }
}

//어닝 컨트롤
//openStatus 값에 움직임
void motorControl(bool openStatus) {
  if(openStatus == true){
    for(deg = 0; deg <= 180; deg++){
      servoWrite(servoChannel, deg);
    }
    client.publish(PUB_TOPIC, "Awning Close");
  } else if(openStatus == false){
    for(deg = 180; deg >= 0; deg--){
      servoWrite(servoChannel, deg);
    }
    client.publish(PUB_TOPIC, "Awning open");
  } 
}

//LED 컨트롤
//ledStatus 값에 움직임
void ledControl(bool ledStatus, bool forceOn){
  if(ledStatus == false && forceOn == false){
    digitalWrite(LED_Y, HIGH);
    client.publish(PUB_TOPIC, "LED On");
  } else if(ledStatus == true && forceOn == false){
    digitalWrite(LED_Y, LOW);
    client.publish(PUB_TOPIC, "LED Off");
  }
}

//서보모터 값
void servoWrite(int ch, int deg){
  duty = map(deg, 0, 180, 1638, 8192);
  ledcWrite(ch, duty);
  delay(15);
}

//센서 값 TOPIC에 PUBLISH
void sendVal(){
  dtostrf(light, 4, 1, lightVal);
  sprintf(sensVal, "light/%s", lightVal);
  client.publish(PUB_TOPIC_LIGHT, sensVal);

  dtostrf(tempC, 4, 1, tempVal);
  sprintf(sensVal, "temp/%s", tempVal);
  client.publish(PUB_TOPIC_TEMP, sensVal);

  dtostrf(windVal, 4, 1, windValue);
  sprintf(sensVal, "wind/%s", windValue);
  client.publish(PUB_TOPIC_WIND, sensVal);
}

void printData() {
  Serial.print("DATA, ");
  Serial.print(light);
  Serial.print(",");
  Serial.print(tempC);
  Serial.print(",");
  Serial.print(windVal);
  Serial.print(",");
  Serial.print(ledStatus);
  Serial.print(",");
  Serial.println(openStatus);
}
