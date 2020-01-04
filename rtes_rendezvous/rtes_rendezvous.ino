#include <ESP8266WiFi.h>
#include <Servo.h>
#ifndef WIFIMODULE
#define WIFISSID "OnePlus 6T" //"rtesgroup11"
#define WIFIPWD  "12345678" //"rtesgroup11"
#endif
const char* ssid     = WIFISSID;
const char* password = WIFIPWD;
Servo servoMotor; //servoMotor for rotating the ultrasonic sensor
#define STOP_RSSI -45  //If signal for RSSI is below this, it will stop the car
#define RSSICOUNT 10 // To take median of these values
#define stop_distance_threshold 20  //threshold for ultrasonic sensor in cm. To check when the vehicle to should stop.
#define move_distance_threshold 20  //threshold for ultrasonic sensor in cm. If the sensor value is greater than this, vehicle will move, otherwise check in other directions

//PWM pins
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

//int forward_count = 0;
bool stop_bot = false;    // Flag to indicate if the vehicle should stop. This is true when the vehicle has reached the beacon.
//int initialSignal = 0;
int prevReading;          // Initial RSSI reading taken before starting the vehicle
int currentReading;       // RSSI readings taken when the vehicle is moving

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

int signalStrength(){
  int rssi[RSSICOUNT];
  int i = 0;
  for(i = 0; i < RSSICOUNT; i++){
    rssi[i] = WiFi.RSSI();
  }
  // Insertion sort to get the median value
  int key,j;
  for (i=1; i<RSSICOUNT+1; i++) 
    {
      key = rssi[i];  
      j = i-1;  
      while(j>=0 && rssi[j]>key) 
      {  
          rssi[j+1] = rssi[j];  
          j = j-1;  
      }
      rssi[j + 1] = key;  
    }
  Serial.print("Median rssi = ");
  if(RSSICOUNT %2 != 0){  //For odd number of readings
    Serial.println(rssi[(RSSICOUNT/2)]);
    return rssi[(RSSICOUNT/2)];     //Return the mean RSSI value
  }
  else{                   //For even number of readings
    Serial.println((rssi[(RSSICOUNT-1/2)] + rssi[(RSSICOUNT+1/2)])/2);
    return (rssi[(RSSICOUNT-1/2)] + rssi[(RSSICOUNT+1/2)])/2;     //Return the mean RSSI value
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servoMotor.attach(SERVO);    //setting pin for the servo motor
  setIOPins();
  Serial.println("Let the Program begin");
  connectToWifi();
  prevReading = signalStrength();   //Take a reading of RSSI before starting to move the vehicle
}

int calculateDistance(){    //Calculate distance for the ultrasonic sensor
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

void turnServo(int angle){    // Turing the servo motor to the specified angle
  servoMotor.write(angle);
  delay(1000);
}
void getUltrasonicReadings(int *ultrasonicReadingsArray){   //Getting ultrasonic readings for 3 directions front, left and right
  ultrasonicReadingsArray[0] = calculateDistance();   //Reading when facing front
  turnServo(0);    //Rotate Servo to face Right
  ultrasonicReadingsArray[1] = calculateDistance();   //Reading when facing right
  turnServo(180);  //Rotate Servo to face Left
  ultrasonicReadingsArray[2] = calculateDistance();   //Reading when facing left
  turnServo(90);   //Back to Original Position
}

//void startCar(){
//  digitalWrite(STNDBY, HIGH);
//}
void stopCar(){             // To stop the car from moving.
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void turnCar(int rotateDirection){    //Turn the car in given direction right, left or 180 degree
  startCar();
  if(rotateDirection == rotateRight){ //Rotate the car right
    analogWrite(PWMA, 1200);
    analogWrite(PWMB, 1000);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    int delayvar = 800;
    while(delayvar > 0){                  //To continuosly check the signal strength and stop the car if reached near the beacon
      if(signalStrength() > STOP_RSSI){
        stop_bot = true;
        break;
      }
      delayvar -= 100;
      delay(100);
    }
  }
  else if(rotateDirection == rotateLeft){ //Rotate the car left
    analogWrite(PWMA, 950);
    analogWrite(PWMB, 800);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    int delayvar = 1000;
    while(delayvar > 0){                //To continuosly check the signal strength and stop the car if reached near the beacon
      if(signalStrength() > STOP_RSSI){
        stop_bot = true;
        break;
      }
      delayvar -= 100;
      delay(100);
    }
  }
  else if(rotateDirection == rotate180){  //Rotate the car in 180 degree
    turnCar(rotateLeft);
    turnCar(rotateLeft);
  }
  stopCar();
}

void moveForward(int distance){
  while(calculateDistance() > stop_distance_threshold)  //If there is no obstacle in front
  {
    
    Serial.println("In Forward");
    analogWrite(PWMA, 650);
    analogWrite(PWMB, 420);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(400); //delay between consecutive readings of ultrasonic sensor
    currentReading = signalStrength();
    Serial.print("RSSI = ");
    Serial.println(currentReading);
    if(currentReading > STOP_RSSI){   //To continuosly check the signal strength and stop the car if reached near the beacon
        stop_bot = true;
        break;
    }
    else if(currentReading - prevReading <= -6){      //If going far away from the beacon, turn left
      Serial.println("Rssi difference, turning left");
      turnCar(rotateLeft);
      break;
    }
  }
  stopCar();          //Obstacle detected so stopping the car (Will take ultrasonic readings for left and right directions)
}

void loop() {
  while(!stop_bot){                     //If destination is not reached, check rssi and move the bot
    if(signalStrength() > STOP_RSSI){
        stop_bot = true;
    }
    int distance[3];                    // Array to take distance values of 3 directions
    getUltrasonicReadings(distance);    // Take distance readings of 3 directions
    Serial.print("Distance Front = ");
    Serial.println(distance[0]);
    Serial.print("Distance Right = ");
    Serial.println(distance[1]);
    Serial.print("Distance Left = ");
    Serial.println(distance[2]);

    Serial.print("prevReading = ");
    Serial.println(prevReading);       // Initially taken in setup(), later taken in moveForward() function
    currentReading = signalStrength(); // Current RSSI reading
    Serial.print("currentReading = ");
    Serial.println(currentReading);
    if(currentReading - prevReading > -6 && distance[0] > move_distance_threshold){ // If difference of RSSI reading is greater than -6 and no obstacle is present in front, move forward
      Serial.println("Moving Forward");
      moveForward(distance[0]);
      prevReading = currentReading;
    }
    else if(distance[1] > move_distance_threshold && distance[1] > distance[2]){   // If there is no obstacle on the right, and more distance available than left, move right
      Serial.println("Turning Right");
      turnCar(rotateRight);                             // To turn car right
    }
    else if(distance[2] > move_distance_threshold){     // If there is no obstacle on the left, move left
      Serial.println("Turning Left");
      turnCar(rotateLeft);                              // To turn car left
    }
    else{
      Serial.println("Turning 180");
      turnCar(rotate180);                               // Turn 180 degree if everything else is blocked
    }
    delay(1000);
  }
  //If bot reaches the destination
  Serial.println("Destination Reached!!!!!!!");
}
