#include "DHT.h"

DHT dht(13, DHT11);

bool openStatus = false;     //어닝 펼쳐짐 여부
bool ledStatus = false;      //LED 켜짐 여부
bool weatherStatus = false;  //폭염, 강풍 등 여부
bool forceOpen = false;       //어닝 강제 펼침 여부
bool forceOn = false;         //LED 강제 켜짐 여부

//light sensor
int Light_Signal = A0;
int light = 0;

//DC Motor
int IN3Pin = 10;
int IN4Pin = 9;
int ENPin = 8;

//LED
int LED_R = 2;
int LED_G = 3;
int LED_B = 4;
int LED_Y = 12;

float temp = 0;

void setup() {

  Serial.begin(9600);

  //LED 핀 설정
  //온도 표시
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  //보안등
  pinMode(LED_Y, OUTPUT);

  dht.begin();

  //DC모터 설정
  pinMode(IN3Pin, OUTPUT);
  pinMode(IN4Pin, OUTPUT);
  analogWrite(ENPin, 255);

  //엑셀 초기화
  Serial.println("CLEARDATA");

  //각 열의 데이터명
  Serial.println("LABEL, Light, Temp, LED, openStatus");
}

void loop() {
  //조도 센서값 저장
  light = map(analogRead(Light_Signal), 0, 1023, 0, 100);

  float temp = dht.readTemperature();

  //서버에서 데이터 수신 시 모터, LED 제어 테스트
  if (Serial.available()) {
    componentTest();
    componentControl();
  }

  //어닝, LED 강제 켜짐
  //escape 전송 시 탈출
  if(forceOpen == true || forceOn == true){
    Serial.println("If you want to escape type escape");
    while(1){
        String esc = Serial.readStringUntil('\n');
        if(esc == "escape"){
          break;
        }
    }
    forceOpen = false;
    forceOn = false;
  }


  //조건에 맞춰 DC 모터, 조명 작동
  //밤
  if (light < 30) {
    digitalWrite(LED_Y, HIGH);
    ledStatus = true;
    printData(temp);

    if (temp >= 0 & temp < 30) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);

    } else if (temp >= 31 & temp < 35) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);

    } else if (temp >= 36) {

      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
    }

    delay(2500);
  }
  //낮
  else if (light >= 31) {
    digitalWrite(LED_Y, LOW);
    ledStatus = false;
    printData(temp);

    if (temp >= 0 & temp < 30) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);

      if (openStatus == true) {
        //닫힘
        motorControl(openStatus);
        delay(1000);
        openStatus = false;
      }

      delay(1000);
    } else if (temp >= 31 & temp < 35 && openStatus == false) {

      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);

      //열림
      motorControl(openStatus);
      openStatus = true;
      delay(1000);

    } else if (temp >= 36 && openStatus == false) {

      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);

      //열림
      motorControl(openStatus);
      openStatus = true;
      delay(1000);
    }

    delay(2500);
  }
}

//LIGHT, TEMP, LED 값 출력
void printData(float temp) {

  Serial.print("DATA, ");
  Serial.print(light);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  Serial.print(ledStatus);
  Serial.print(",");
  Serial.println(openStatus);
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
    digitalWrite(LED_Y, HIGH);
    delay(2500);
    digitalWrite(LED_Y, LOW);
    delay(2500);
    Serial.println("LED TEST DONE");
  }
}

//시리얼 통신을 통해 특정 단어 입력 시 컴포넌트 조작
void componentControl() {
  String component = Serial.readStringUntil('\n');

  //open 입력 시 어닝 펼침
  //이후 yes 입력 시 탈출
  if (component == "pen") {

    if (openStatus == true) {
      Serial.println("Smart Shade is already Opened");
    } else if (openStatus == false) {
      Serial.println("Smart Shade Open");
      motorControl(openStatus);
      openStatus = true;
      forceOpen = true;      
    }

    //close 입력 시 어닝 접음
    //이후 yes 입력 시 탈출
  } else if (component == "lose") {

    if (openStatus == false) {
      Serial.println("Smart Shade is already Closed");
    } else if (openStatus == false && forceOpen == false) {
      Serial.println("Smart Shade Close");
      motorControl(openStatus);
      openStatus = false;
      forceOpen = true;
    }

  }

  //on 입력 시 LED 작동
  //이후 yes 입력 시 탈출
  if (component == "n") {

    if (ledStatus == true) {
      Serial.println("LED is already ON");
    } else if (ledStatus == false) {
      Serial.println("LED ON");
      digitalWrite(LED_Y, HIGH);
      ledStatus = true;
      forceOn = true;
    }

    //off 입력 시 LED 꺼짐
    //이후 yes 입력 시 탈출
  } else if (component == "ff") {

    if (ledStatus == false) {
      Serial.println("LED is already OFF");
    } else if (ledStatus == true) {
      Serial.println("LED OFF");
      digitalWrite(LED_Y, LOW);
      ledStatus = false;
      forceOn = true;
    }
  }
}

//어닝 컨트롤
//openStatus 값에 움직임
void motorControl(bool openStatus) {
  if(openStatus == true){
    digitalWrite(IN3Pin, HIGH);
    digitalWrite(IN4Pin, LOW);
    delay(5000);
    digitalWrite(IN3Pin, LOW);
    digitalWrite(IN4Pin, LOW);
    Serial.println("test1");
  } else if(openStatus == false){
    digitalWrite(IN3Pin, LOW);
    digitalWrite(IN4Pin, HIGH);
    delay(5000);
    digitalWrite(IN3Pin, LOW);
    digitalWrite(IN4Pin, LOW);
    Serial.println("test2");
  } 
}