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
#define ULTRA_MIN_DISTANCE 4.28 // in cm
#define TURNING_TIME_MS 330 // The time duration (ms) for turning
#define RGB_WAIT 200 //milliseconds
#define LDR_WAIT 10 //milliseconds
#define IR_THRESHOLD 860 // if the robot is too close, input will be higher than threshold
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
uint16_t r_values[7] = {340, 208, 327, 227, 243, 335, 270};
uint16_t g_values[7] = {611, 382, 435, 489, 540, 494, 478};
uint16_t b_values[7] = {513, 301, 354, 352, 484, 361, 433};
int values[3] = {0, 0, 0};

void setup() {
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  delay(100);
  toggle_led(OFF_LED);
  
  led.setpin(13);
  pinMode(IR, INPUT);
  pinMode(LDR, INPUT);
  delay(1000);
  Serial.begin(9600);
  delay(1000);

  
}
void loop() {
  pid_move(10);
  /*
  toggle_led(RED_LED);
  Serial.println(analogRead(IR));
  if (analogRead(IR) >= IR_THRESHOLD) {
    leftMotor.run(-220);
    rightMotor.run(255);
  }
  else {
    pid_move(10);
  }
  */
  delay(10);
  /*
  Serial.println(ultra_read());
  /*
  int16_t colour = check_colour();
  Serial.println(colour);
  delay(1000);
  */
  /*
  led_green();
  Serial.println(analogRead(LDR));
  delay(10);
  */

  

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
    return RED;
  }
  else if (is_colour(GREEN)) {
    return GREEN;
  }
  else if (is_colour(BLUE)) {
    return BLUE;
  }
  else if (is_colour(ORANGE)) {
    return ORANGE;
  }
  else if (is_colour(PURPLE)) {
    return PURPLE;
  }
  else if (is_colour(WHITE)) {
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
  uint16_t total =0;
  for(uint16_t i = 0; i < times; i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDR_WAIT); // LDR Delay
  }
  return total/times;
}

void pid_move(float target_distance) {
  float curr_distance = ultra_read();
  float error = (target_distance - curr_distance) / target_distance; // if robot is too near the wall, error is +ve, else -ve
  float left = -(255 + kP * 255 * error);
  float right = 255 - kP * 255 * error;
  
  if (curr_distance == 0) {
    leftMotor.run(-255);
    rightMotor.run(230);
  }
  else {
    leftMotor.run((int16_t)left);
    rightMotor.run((int16_t)right);
  }
}
