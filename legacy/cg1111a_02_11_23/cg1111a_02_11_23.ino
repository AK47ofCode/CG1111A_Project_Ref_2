#include "MeMCore.h"

// Ports
#define IR 2
#define LDR 3
#define DECODER_A 9
#define DECODER_B 10
#define ULTRASONIC 12

// Constants
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRA_MIN_DISTANCE 3.72 // in cm
#define ULTRA_MAX_DISTANCE 11 // in cm (max ~18cm)
#define ULTRA_TRACKING_DISTANCE 10 //in cm
#define TURNING_TIME 320 // The time duration (ms) for turning
#define FORWARD_TIME 700 // The time duration (ms) for turning
#define RGB_WAIT 200 //milliseconds
#define LDR_WAIT 10 //milliseconds
#define IR_THRESHOLD 60// if the robot is too close, input will be higher than threshold
#define kP 2
#define kD 0

// RGB Colour ID
#define WHITE 0
#define BLACK 1
#define RED 2
#define GREEN 3
#define BLUE 4
#define ORANGE 5
#define PURPLE 6
#define THRESHOLD 45

// LED ID
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2
#define OFF_LED 3

MeDCMotor leftMotor(M1); // Forward is -255
MeDCMotor rightMotor(M2); // Forward is 255
MeLineFollower lineFinder(PORT_4);
MeRGBLed led(0,30);
MeBuzzer buzzer;

//WHITE, BLACK, RED, GREEN, BLUE, ORANGE, PURPLE
uint16_t r_values[7] = {330, 208, 298, 209, 215, 327, 263};
uint16_t g_values[7] = {611, 382, 455, 537, 553, 536, 521};
uint16_t b_values[7] = {513, 301, 347, 366, 473, 378, 446};
int values[3] = {0, 0, 0};

void setup() {
  pinMode(A7, INPUT);
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  delay(100);
  toggle_led(OFF_LED);
  
  led.setpin(13);
  pinMode(IR, INPUT);
  pinMode(LDR, INPUT);
  //delay(1000);
  Serial.begin(9600);
  //delay(1000);
  while(analogRead(A7) > 100){}
  
}
void loop() {
  //Serial.println(ultra_read());

  //Serial.println(analogRead(IR));
  if (lineFinder.readSensors() == S1_IN_S2_IN) {
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    Serial.println(check_colour());
  }
  else{
    //
    if (analogRead(IR) < IR_THRESHOLD && ultra_read() > 14) {
      leftMotor.run(-200);
      rightMotor.run(255);
    }
    else {
      pid_move();
    }
  }
  delay(10);
  
}

void toggle_led(uint16_t colour) {
  switch(colour) {
    case OFF_LED:
      digitalWrite(DECODER_A, LOW);
      digitalWrite(DECODER_B, LOW);
      break;

    case RED_LED:
      digitalWrite(DECODER_A, HIGH);
      digitalWrite(DECODER_B, LOW);
      break;

    case GREEN_LED:
      digitalWrite(DECODER_A, LOW);
      digitalWrite(DECODER_B, HIGH);
      break;

    case BLUE_LED:
      digitalWrite(DECODER_A, HIGH);
      digitalWrite(DECODER_B, HIGH);
      break;
  }
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

bool is_colour(uint16_t colour) {
  return ((abs(values[0] - r_values[colour]) <= THRESHOLD) && (abs(values[1] - g_values[colour]) <= THRESHOLD) && (abs(values[2] - b_values[colour]) <= THRESHOLD));
}

int16_t check_colour() {
  Serial.println("START");
 
  for (uint16_t i = 0; i < 3; i += 1) {
    toggle_led(i);
    delay(RGB_WAIT);
    values[i] = getAvgReading(5);
    delay(RGB_WAIT);
  }
  toggle_led(OFF_LED);
  for (int i = 0; i < 3; i += 1) {
    Serial.println(values[i]);
  }
  if (is_colour(RED)) {
    leftMotor.run(255);
    rightMotor.run(255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    return RED;
  }
  else if (is_colour(GREEN)) {
    leftMotor.run(-255);
    rightMotor.run(-255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    return GREEN;
  }
  else if (is_colour(BLUE)) {
    leftMotor.run(-255);
    rightMotor.run(-255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    leftMotor.run(-255);
    rightMotor.run(255);
    delay(FORWARD_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    leftMotor.run(-255);
    rightMotor.run(-255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    return BLUE;
  }
  else if (is_colour(ORANGE)) {
    leftMotor.run(-255);
    rightMotor.run(-255);
    delay(2 * TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    return ORANGE;
  }
  else if (is_colour(PURPLE)) {
    leftMotor.run(255);
    rightMotor.run(255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    leftMotor.run(-255);
    rightMotor.run(255);
    delay(FORWARD_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    leftMotor.run(255);
    rightMotor.run(255);
    delay(TURNING_TIME);
    leftMotor.run(0);
    rightMotor.run(0);
    delay(500);
    return PURPLE;
  }
  else if (is_colour(WHITE)) {
    buzzer.tone(392, 200);
    buzzer.tone(523, 200);
    buzzer.tone(659, 200);
    buzzer.tone(784, 200);
    buzzer.tone(659, 150);
    buzzer.tone(784, 400);
    buzzer.noTone();
    return WHITE;
  }
  else if (is_colour(BLACK)) {
    return BLACK;
  }
  return -1;
}

bool is_on_black_line() {
  int sensorState = lineFinder.readSensors();
  if (sensorState == S1_IN_S2_IN) {
    buzzer.tone(392, 200);
  }
  delay(100);
}

uint16_t getAvgReading(uint16_t times){      
  uint16_t reading;
  uint16_t total = 0;
  for(uint16_t i = 0; i < times; i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDR_WAIT); // LDR Delay
  }
  return total/times;
}

void pid_move() {
  float curr_distance = ultra_read();
  if (curr_distance == 0 || curr_distance >= ULTRA_MAX_DISTANCE) {
    leftMotor.run(-255);
    rightMotor.run(255);
  }
  else {
    float error = (ULTRA_TRACKING_DISTANCE - curr_distance) / ULTRA_TRACKING_DISTANCE; // if robot is too near the wall, error is +ve, else -ve
    //float left = -(255 + kP * 255 * error);
    float left = -(255);
    float right = 255 - kP * 255 * error;
    leftMotor.run((int16_t)left);
    rightMotor.run((int16_t)right);
  }
}
