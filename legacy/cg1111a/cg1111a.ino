#include "MeMCore.h"

#define IR_SENSOR 2
#define DECODER_A 9
#define DECODER_B 10
#define ULTRASONIC 12

#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRA_MIN_DISTANCE 3.72 // in cm
#define ULTRA_MAX_DISTANCE 16.6 // in cm
#define ULTRA_TRACKING_DISTANCE 8.5 //in cm
#define TURNING_TIME_MS 330 // The time duration (ms) for turning
#define kP 1
#define kD 0

MeDCMotor leftMotor(M1); // Forward is -255
MeDCMotor rightMotor(M2); // Forward is 255
MeLineFollower lineFinder(PORT_4);
MeRGBLed led(0,30);

void setup() {
  // Any setup code here runs only once:
  //delay(10000); // Do nothing for 10000 ms = 10 seconds
  /*
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  delay(1000);
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  

  led.setpin(13);
  
  pinMode(IR_SENSOR, INPUT);
  delay(1000);
  */
  Serial.begin(9600);
  delay(1000);
}
void loop() {

  
  Serial.print("distance(cm) = ");
  Serial.println(ultra_read());
  
  //Serial.println(analogRead(IR_SENSOR));
  //Serial.println(analogRead(3));
  //delay(10);
  /*
  if (analogRead(IR_SENSOR) >= 300 && analogRead(IR_SENSOR) <= 450) {
    led.setColor(0, 255, 0);
    led.show();
  
  }
  else {
    led.setColor(0, 0, 255);
    led.show();
  }
  
  
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  delay(1000);
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  delay(1000);
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  delay(1000);
  digitalWrite(I1, HIGH);
  digitalWrite(I2, HIGH);
  delay(1000);
  */
  delay(10);
}

float ultra_read() {
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  return duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100; 
}

void pid_move() {
  
}
