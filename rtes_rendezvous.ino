#include <ESP8266WiFi.h>
#include <Servo.h>
#ifndef WIFIMODULE
#define WIFISSID "rtesgroup11"//"OnePlus 6T"
#define WIFIPWD  "rtesgroup11"//"12345678"
#endif
const char* ssid     = WIFISSID;
const char* password = WIFIPWD;
Servo servoMotor; //servoMotor for rotating the ultrasonic sensor
#define STOP_RSSI -50  //If signal for RSSI is below this, it will stop the car
#define stop_distance_threshold 20
#define move_distance_threshold 30
//Motor A
#define SERVO 14 // Servo
#define PWMA 5 //Speed control
#define AIN1 0 //Direction wheel A
#define AIN2 15 //Direction wheel A
//Motor B
#define PWMB 4 //Speed control
#define BIN1 16 //Direction wheel B
#define BIN2 2 //Direction wheel B
//#define STNDBY 0 //Standby pin
// Trig and Echo pin for ultrasonic:
#define trigPin 12
#define echoPin 13

#define rotateRight 1
#define rotateLeft 2
#define rotate180 0

int forward_count = 0;
bool stop_bot = false;

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
  connectToWifi();
}

int signalStrength(){
  long rssi = WiFi.RSSI();
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
  turnServo(0);    //Rotate Servo to face Right
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
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(1200);
  }
  if(rotateDirection == rotateLeft){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(1200);
  }
  if(rotateDirection == rotate180){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(2000);
  }
  stopCar();
}

void moveReverse(){
  Serial.println("Moving Reverse");
  analogWrite(PWMA, 550);
  analogWrite(PWMB, 400);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  delay(500);
  stopCar();
}
int moveForward(int distance){
//  startCar();
  Serial.println("Moving Forward");
  int stuckCount = 50;  //Currently set for 50x400ms(delay in the loop below) = 20seconds
  analogWrite(PWMA, 550);
  analogWrite(PWMB, 400);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  while(calculateDistance() > stop_distance_threshold && stuckCount > 0){
    delay(400); //delay between consecutive readings of ultrasonic sensor
    stuckCount --;
    Serial.print("RSSI = ");
    Serial.println(signalStrength());
    if(signalStrength() > STOP_RSSI){
        stop_bot = true;
        break;
    }
  }
  if(stuckCount<=0){
    return 1; //Car is stuck
  }
  stopCar();
  return 0;
}

void loop() {
  if(!stop_bot){  //If destination is not reached, check rssi and move the bot
    if(signalStrength() > STOP_RSSI){
        stop_bot = true;
    }
    int distance[3];
    getUltrasonicReadings(distance);
    Serial.print("Distance Front = ");
    Serial.println(distance[0]);
    Serial.print("Distance Right = ");
    Serial.println(distance[1]);
    Serial.print("Distance Left = ");
    Serial.println(distance[2]);
    if(distance[0] > move_distance_threshold && forward_count <= 3){
      int stuckFlag = moveForward(distance[0]);
      if(stuckFlag){
        forward_count = 4;  //Need to stop the car
        moveReverse();
      }
      forward_count++;
    }
    else if(distance[1] > move_distance_threshold && distance[1] > distance[2]){
      Serial.println("Turning Right");
      turnCar(rotateRight);
      forward_count = 0;
  //    moveForward(distance[1]);
    }
    else if(distance[2] > move_distance_threshold){
      Serial.println("Turning Left");
      turnCar(rotateLeft);
      forward_count = 0;
  //    moveForward(distance[2]);
    }
    else{
      Serial.println("Turning 180");
      turnCar(rotate180);
    }
    delay(2000);
  }
  else{ //If bot reaches the destination
    Serial.println("Destination Reached!!!!!!!");
  }
}
