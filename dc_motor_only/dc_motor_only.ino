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


void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // pinMode(trigPin, OUTPUT);
  // pinMode(echoPin, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // pinMode(SERVO, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
//  startCar();
  analogWrite(PWMA, 300);
  analogWrite(PWMB, 350);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  // while(calculateDistance() > distance_threshold);
  // stopCar();
}
