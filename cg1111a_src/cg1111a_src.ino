#include "MeMCore.h"
#include <math.h>

// Ports
#define IR 2
#define LDR 3
#define DECODER_A 9
#define DECODER_B 10
#define ULTRASONIC 12

// Constants
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define RGB_WAIT 50 //milliseconds
#define LDR_WAIT 10 //milliseconds

// Movement
#define TURNING_TIME 305 // The time duration (ms) for turning
#define FORWARD_TIME 700 // The time duration (ms) for turning
#define MOVE_WAIT 50

#define ULTRA_LOWER_RANGE 11.5 // in cm hihihi
#define ULTRA_UPPER_RANGE 15.8 // in cm (max ~18cm)
#define ULTRA_TARGET 10 //in cm
#define kP_ULTRA 0.7
#define kD_ULTRA 3.1
int16_t prev_error_ultra = 0;

#define IR_LIMIT 300 // value before the readings plateau
#define IR_TARGET 200 // if the robot is too close, input will be higher than threshold
#define kP_IR 2.2
#define kD_IR 0.8
int16_t prev_error_IR = 0;

// LED ID
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2
#define OFF_LED 3

// RGB Colour ID
#define BLACK 0
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLUE 4
#define ORANGE 5
#define PURPLE 6
#define THRESHOLD 45

//BLACK, WHITE, RED, GREEN, BLUE, ORANGE, PURPLE
int16_t r_values[7] = {0, 313, 298, 155, 180, 307, 216};
int16_t g_values[7] = {0, 628, 435, 502, 553, 501, 489};
int16_t b_values[7] = {0, 556, 399, 408, 528, 402, 479};
int16_t values[3] = {0, 0, 0};

MeDCMotor leftMotor(M1); // Forward is -255
MeDCMotor rightMotor(M2); // Forward is 255
MeLineFollower lineFinder(PORT_4);
MeRGBLed led(0,30);
MeBuzzer buzzer;

void mbot_led(int16_t r = 0, int16_t g = 0, int16_t b = 0) {
    led.setColor(r, g, b);
    led.show();
}

void toggle_led(int16_t colour) {
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

void move(int16_t left, int16_t right, int16_t wait = 0) {
    leftMotor.run(-left);
    rightMotor.run(right);
    delay(wait);
}

void PID_ultra() {
  float left = 255;
  float right = 255;
  float curr_distance = ultra_read();
  if (curr_distance != 0) {
    // if robot is too near the wall, error is +ve, else -ve
    float error = (ULTRA_TARGET - curr_distance) / ULTRA_TARGET; 
    float output = kP_ULTRA * error + kD_ULTRA * (error - prev_error_ultra);
    left = 255 * (1 + output);
    right = 255 * (1 - output); 
    prev_error_ultra = error;
  }
  move(left, right);
}

void PID_IR() {
  float left = 255;
  float right = 255;
  float curr_distance = analogRead(IR);
  if (curr_distance != 0) {
    // if robot is too near the wall, error is +ve, else -ve
    float error = (IR_TARGET - curr_distance) / IR_TARGET; 
    float output = kP_IR * error + kD_IR * (error - prev_error_IR);
    left = 255 * (1 - output);
    right = 255 * (1 + output); 
    prev_error_IR = error;
  }
  move(left, right);
}

bool is_colour(int16_t colour) {
  return ((abs(values[0] - r_values[colour]) <= THRESHOLD) && (abs(values[1] - g_values[colour]) <= THRESHOLD) && (abs(values[2] - b_values[colour]) <= THRESHOLD));
}

int16_t getAvgReading(int16_t times){      
  int16_t reading;
  int16_t total = 0;
  for(int16_t i = 0; i < times; i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDR_WAIT); // LDR Delay
  }
  return total/times;
}

int16_t check_colour() { //k-NN algorithm
  Serial.println("START");
  for (int16_t i = 0; i < 3; i += 1) {
    toggle_led(i);
    delay(RGB_WAIT);
    values[i] = getAvgReading(5);
    delay(RGB_WAIT);
  }
  toggle_led(OFF_LED);  
  for (int16_t i = 0; i < 3; i += 1) {
    Serial.println(values[i]);
  }
  double min_distance;
  int16_t colour = BLACK;
  for (int16_t i = 0; i < 7; i += 1) {
      double distance = 0;
      distance += fabs(r_values[i] - values[0]) * fabs(r_values[i] - values[0]);
      distance += fabs(g_values[i] - values[1]) * fabs(g_values[i] - values[1]);
      distance += fabs(b_values[i] - values[2]) * fabs(b_values[i] - values[2]);
      distance = sqrt(distance);
      if (i == 0) {
        min_distance = distance;
      }
      else {
        if (distance < min_distance) {
        min_distance = distance;
        colour = i;
        }
      }
  }
  return colour;
}

void overall_movement() {
  float ultra = ultra_read();
  mbot_led();
  if (ultra > 0.00 && ultra <= ULTRA_LOWER_RANGE) {
    //beyond ULTRA_LOWER_RANGE, closer to the left
    mbot_led(255, 0, 0);
    PID_ultra();
  }
  else if (ultra >= ULTRA_UPPER_RANGE) {
    //beyond ULTRA_UPPER_RANGE, closer to the right
    mbot_led(0, 0, 255);
    if (analogRead(IR) <= IR_LIMIT) {
      PID_IR();
    }
  }
  else {
    //between ULTRA_LOWER_RANGE and ULTRA_UPPER_RANGE || ultra > 25
    mbot_led(0, 255, 0);
    move(255, 255);
  }
}

void setup() {
  Serial.begin(9600);
  led.setpin(13);
  pinMode(IR, INPUT);
  pinMode(LDR, INPUT);
  pinMode(A7, INPUT);
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  toggle_led(OFF_LED);
  mbot_led();
  move(0, 0);
  while(analogRead(A7) > 100){} //wait for button press
}
void loop() {
  //Serial.println(ultra_read());
  //Serial.println(analogRead(IR));
  //check_colour();
  if (lineFinder.readSensors() == S1_IN_S2_IN) {
    move(0, 0);
    mbot_led();
    switch(check_colour()) {
      case WHITE:
        mbot_led(255, 255, 255);
        buzzer.tone(392, 200);
        buzzer.tone(523, 200);
        buzzer.tone(659, 200);
        buzzer.tone(784, 200);
        buzzer.tone(659, 150);
        buzzer.tone(784, 400);
        buzzer.noTone();
        break;
      case RED:
        mbot_led(255, 0, 0); 
        move(-255, 255, TURNING_TIME);
        move(0, 0, MOVE_WAIT);
        break;
      case GREEN:
        mbot_led(0, 255, 0); 
        move(255, -255, TURNING_TIME);
        move(0, 0, MOVE_WAIT);
        break;
      case BLUE:
        mbot_led(0, 0, 255); 
        move(255, -255, TURNING_TIME);
        move(0, 0, MOVE_WAIT);
        move(255, 255, FORWARD_TIME + 80);
        move(255, -255, TURNING_TIME + 100);
        move(0, 0, MOVE_WAIT);
        break;
      case ORANGE:
        mbot_led(255, 165, 0); 
        move(-255, 255, TURNING_TIME * 2 - 20);
        move(0, 0, MOVE_WAIT);
        break;
      case PURPLE:
        mbot_led(128, 0, 128); 
        move(-255, 255, TURNING_TIME);
        move(0, 0, MOVE_WAIT);
        move(255, 255, FORWARD_TIME - 30);
        move(-255, 255, TURNING_TIME + 100);
        move(0, 0, MOVE_WAIT);
        break;
    }
    mbot_led();
  }
  else {
    overall_movement();
  }
  delay(10);
}
