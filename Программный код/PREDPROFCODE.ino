#include <Servo.h>
#include <arduino_encoder.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#define THRESHOLD 500



#define LEFT_SENSOR A0   // Левый датчик (по часовой стрелке)
#define CENTERLEFT_SENSOR A1
#define CENTERRIGHT_SENSOR A2
#define RIGHT_SENSOR A3  // Правый датчик (против часовой стрелки)


Servo claw;
Encoder enc1;
Encoder enc2;
int flagL = 0;
int flagR = 0;
int L = 90;
int R = 90;
int cycles = 1;
int dist[5]; int distM[5];
int Sold = 0;
int Step = 1;
int v = 90;
int globalRotations = 0;
float Kp = 0.5, Ki = 0.002, Kd = 0.4;
int BASE_SPEED = 90;
int lastError = 0, integral = 0;
long startTime;
long actionTime;
float distFilt;
void setup() {
  for (int i = 2; i <= 5; i++) pinMode(i, OUTPUT);
  pinMode(11, OUTPUT); pinMode(10, INPUT);
  long startTime = millis();
  enc1.setup(7, 6);
  enc2.setup(9, 8);
  Serial.begin(9600);
  claw.attach(12);
  claw.write(0);
}

void go(int L, int R) { // ПРООСТОЕ ДВИЖЕНИЕ
  if (L > 0) {
    analogWrite(3, L); digitalWrite(2, 1);
  }
  else {
    analogWrite(3, abs(L)); digitalWrite(2, 0);
  }
  if (R > 0) {
    analogWrite(5, R); digitalWrite(4, 1);
  }
  else {
    analogWrite(5, abs(R)); digitalWrite(4, 0);
  }
}

void go_enc() {   
  int E = abs(enc1.get()) - abs(enc2.get());
  int m1 = abs(v) + E * 1; m1 = constrain(m1, -110, 110);
  int m2 = abs(v) - E * 1; m2 = constrain(m2, -110, 110);
  
//  int leftLineValue = analogRead(lineSensorLeftPin);
//  int rightLineValue = analogRead(lineSensorRightPin);
//  Serial.println(leftLineValue);
//  Serial.println(rightLineValue);
  
  
  if (v > 0){
    go(m1, m2);
  }
  else{
    go(-m1, -m2);
  }
}
void go_encN(int N) {
  int x = N * 1.1666666667;
  enc1.clear();
  enc2.clear();
  while (abs(enc1.get()) < x || abs(enc2.get()) < x) {
    go_enc();
  }
  go(-v, -v); delay(50);
  go(0, 0); delay(1000);
    //Serial.println(enc1.get());
}

void counterClockwise(){
  int sensor = analogRead(RIGHT_SENSOR);  // Правый датчик
  
  int error = sensor - THRESHOLD;
  
  // ПИД-регулятор
  integral += error;
  int derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  
  // Расчет скоростей
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;
  
  // Ограничение скоростей
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);
  
  // Управление моторами
  go(leftSpeed, rightSpeed);
  
  delay(10);
}

void clockwise(){
  int sensor = analogRead(LEFT_SENSOR) * 5;  // Левый датчик
  
  // Для левого датчика, возможно, нужно инвертировать ошибку
  // или использовать другой THRESHOLD_LEFT
  int error = sensor - THRESHOLD;  // Возможно, нужно: THRESHOLD - sensor
  
  // ПИД-регулятор
  integral += error;
  int derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  
  // Расчет скоростей
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;
  
  // Ограничение скоростей
  leftSpeed = constrain(leftSpeed, -80, 80);
  rightSpeed = constrain(rightSpeed, -80, 80);
  
  // Управление моторами
  go(leftSpeed, rightSpeed);
  
  
}




long leftFirstSeenLine;
long rightFirstSeenLine;

int leftMoveFlag;
int rightMoveFlag;


void gotoTheLine(){
  while (analogRead(CENTERLEFT_SENSOR) >= 100 and analogRead(CENTERRIGHT_SENSOR) >= 100){
    go_enc();
  }
  go(-v, -v);
  delay(50);
  go(0, 0);
}

void connectToTheLineClockwise(){
  while (analogRead(LEFT_SENSOR) >= 100){
    go_enc();
  }
  go(-v, -v);
  delay(50);
  go(0, 0);
}


int uzd() {
  // импульс 10 мкс
  if (millis() - actionTime < 200){
    return distFilt;
  }
  digitalWrite(11, 1);
delayMicroseconds(10);
  digitalWrite(11, 0);
   
  // измеряем время ответного импульса
  uint32_t us = pulseIn(10, HIGH);
  float dist = us / 58.2;   // получаем расстояние
  distFilt += (dist - distFilt) * 0.2;      // фильтруем
  // считаем расстояние и возвращаем
  return distFilt;
}
void left(int a) {
  if (((globalRotations % 8) > 3) and ((globalRotations % 8) < 6)){
    a = a / 1.33;
  }
  else if ((globalRotations % 8) >= 6){
    a = a * 1.1;
  }
  int x = map(a, 0, 360, 0, 900);
  enc1.clear();
  enc2.clear();
  while (abs(enc1.get()) < x || abs(enc2.get()) < x) {
    go(-v * 1.1, v * 1.1);
    
  }
  go(v, -v); delay(50);
  go(0, 0); delay(1000);
  globalRotations++;
}

void Bancan_1() {
  Step++;
}
long leftFirstSeenTime = 0;
long rightFirstSeenTime = 0;
void loop() {
 // clockwise();
 Serial.println(uzd());
  if (cycles <= 4 and false) {

    switch (Step){
    case 1:
      claw.write(0);
      left(45);
      Step++;
      break;
    case 2:
      go(0,0);
      if (cycles >= 2){
        go_encN(330);
      }
      else{
        go_encN(280);
      }
      Step++;
      break;
    case 3:
      for (int i = 0; i < 90; i++){
        claw.write(i);
        delay(10);
      }
      Step++;
     break;
    
    case 4:
      v = v * -1;
      if (cycles >= 2){
        go_encN(550);
      }
      else{
        go_encN(280);
      }
      
      Step++;
      break;
    case 5:
      left(45);
      Step++;
      break;
    case 6:
      v = v * -1;
      enc1.clear();
      enc2.clear();
      gotoTheLine();
      Step++;
      delay(1000);
      break;
    case 7:
      claw.write(0);
      Step++;
      break;  
    case 8:
      v = v * -1;
      if (cycles >= 2){
        go_encN(470);
      }
      else{
        go_encN(420);
      }
      Step++;
      delay(5000);
      break;   
    case 9:
      
      
      left(45);
      Step = 2;
      cycles++;
      v = abs(v);
      delay(3000);
    }
  }
  else{
    
    enc1.clear();
    enc2.clear();
    delay(1500);
    v = 75;
    gotoTheLine();
    delay(1500);
    v = 90;
    v = v * -1;
    go_encN(80);
    v = abs(v);
    delay(1500);
    go(v, -v);
    delay(300);
    go(0,0);
    enc1.clear();
    enc2.clear();
    
    delay(1800);
    enc1.clear();
    enc2.clear();
    
    gotoTheLine();
    delay(1000);
    go(0, 0);
    delay(500);
    v = abs(v);
    lastError = 0;
    integral = 0;
    startTime = millis();
    while (true){
    clockwise();
    if (millis() - startTime > 5000 and uzd() < 5){
      if (uzd() < 5){
        if (uzd() < 5){
          break;
        }
      }
    }
    }
    go(-v, -v);
    delay(50);
    go(0, 0);
     for (int i = 0; i < 90; i++){
        claw.write(i);
        delay(10);
      }
    
    v = v * -1;
    left(45);
    claw.write(0);
    delay(600);
    v = v * -1;
    left(45);
    delay(1300);
    connectToTheLineClockwise();
    delay(750);
    while (true){
    clockwise();
    }
}
}
