#include <ESP8266WiFi.h>
#include <Servo.h>
#ifndef WIFIMODULE
#define WIFISSID "OnePlus 6T"
#define WIFIPWD  "12345678"
#endif
const char* ssid     = WIFISSID;
const char* password = WIFIPWD;
Servo servoMotor; //servoMotor for rotating the ultrasonic sensor
#define STOP_RSSI 30  //If signal for RSSI is below this, it will stop the car
#define distance_threshold 30
//Motor A
#define SERVO 0 // Servo
#define PWMA 5 //Speed control
#define AIN1 13 //Direction wheel A
#define AIN2 15 //Direction wheel A
//Motor B
#define PWMB 2 //Speed control
#define BIN1 12 //Direction wheel B
#define BIN2 14 //Direction wheel B
//#define STNDBY 0 //Standby pin
// Trig and Echo pin for ultrasonic:
#define trigPin 4
#define echoPin 16

#define rotateRight 1
#define rotateLeft 2
#define rotate360 0

void setIOPins(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(SERVO, OUTPUT);
} 
void connectToWifi(){
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(WiFi.status());
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servoMotor.attach(SERVO);
  setIOPins();
  Serial.println("Let the Program begin");
  //connectToWifi();
}

int signalStrength(){
  long rssi = -WiFi.RSSI();
  Serial.print("rssi = ");
  Serial.println(rssi);
  return rssi;
}

int calculateDistance(){
  long duration;
  int distance;
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance:
  distance= duration*0.034/2;
  Serial.print("Distance = ");
  Serial.println(distance);
  return distance;
}

void turnServo(int angle){
  servoMotor.write(angle);
  delay(1000);
}
void getUltrasonicReadings(int *ultrasonicReadingsArray){
  ultrasonicReadingsArray[0] = calculateDistance();
  turnServo(10);    //Rotate Servo to face Right
  ultrasonicReadingsArray[1] = calculateDistance();
  turnServo(180);  //Rotate Servo to face Left
  ultrasonicReadingsArray[2] = calculateDistance();
  turnServo(90);   //Back to Original Position
}

void startCar(){
  //digitalWrite(STNDBY, HIGH);
}
void stopCar(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void turnCar(int rotateDirection){
  startCar();
  if(rotateDirection == rotateRight){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(300);
  }
  if(rotateDirection == rotateLeft){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(300);
  }
  if(rotateDirection == rotate360){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(500);
  }
  stopCar();
}

void moveForward(int distance){
  startCar();
  analogWrite(PWMA, 300);
  analogWrite(PWMB, 350);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  while(calculateDistance() > distance_threshold);
  stopCar();
}

void loop() {
  // put your main code here, to run repeatedly:
//  if(signalStrength() < STOP_RSSI){
//    while(1){
//      Serial.println("Destination Reached!!!!!!!");
//      delay(1000);
//    }
//  }
  int distance[3];
  getUltrasonicReadings(distance);
  Serial.print("Distance Front = ");
  Serial.println(distance[0]);
  Serial.print("Distance Right = ");
  Serial.println(distance[1]);
  Serial.print("Distance Left = ");
  Serial.println(distance[2]);
  if(distance[0] > distance_threshold){
    moveForward(distance[0]);
    Serial.println("Moving Forward");
  }
  else if(distance[1] > distance_threshold){
    Serial.println("Turning Right");
    turnCar(rotateRight);
//    moveForward(distance[1]);
  }
  else if(distance[2] > distance_threshold){
    Serial.println("Turning Left");
    turnCar(rotateLeft);
//    moveForward(distance[2]);
  }
  else{
    delay(1000);
    Serial.println("Turning 360");
    turnCar(rotate360);
  }
  delay(5000);
}
