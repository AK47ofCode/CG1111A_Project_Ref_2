#include "MeMCore.h"
#include <math.h>

/**
 * Ports
 * IR -> A2
 * LDR -> A3
 * 1A of Decoder -> D9
 * 1B of Decoder -> D10
 * Ultrasonic Sensor -> D12
 */
#define IR 2
#define LDR 3
#define DECODER_A 9
#define DECODER_B 10
#define ULTRASONIC 12

/**
 * Ultrasonic Sensor Constants
 */
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340

/**
 * Colour Sensor Constants
 */
#define RGB_WAIT 50 //milliseconds
#define LDR_WAIT 10 //milliseconds

/**
 * Movement Constants
 */
#define TURNING_TIME 280 // The time duration (ms) for turning
#define FORWARD_TIME 705 // The time duration (ms) for turning
#define MOVE_WAIT 50
#define ULTRA_LOWER_RANGE 11.0 // in cm
#define ULTRA_UPPER_RANGE 14.8 // in cm; max ~18cm
#define ULTRA_TARGET 10.0 // in cm; target value for PID (Ultrasonic Sensor)
#define IR_LIMIT 135 // PID correction for IR triggers if analogRead(IR) <= IR_LIMIT
#define IR_TARGET 120 // target value for PID (IR)

/**
 * PID Constants / Variables
 */
#define kP_ULTRA 0.65
#define kD_ULTRA 2.7
#define kP_IR 1.6
#define kD_IR 0.5
int16_t prev_error_ultra = 0;
int16_t prev_error_IR = 0;

/**
 * Colour Sensor LED IDs
 */
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2
#define OFF_LED 3

/**
 * Colour IDs
 */
#define BLACK 0
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLUE 4
#define ORANGE 5
#define PURPLE 6
#define THRESHOLD 45

/**
 * RGB Values
 * r_values[7] -> LDR measurements for red component of each colour
 * g_values[7] -> LDR measurements for red component of each colour
 * b_values[7] -> LDR measurements for red component of each colour
 * values[3] -> current RGB values
 */
const int16_t r_values[7] = {0, 292, 271, 145, 142, 296, 189};
const int16_t g_values[7] = {0, 652, 477, 536, 580, 541, 524};
const int16_t b_values[7] = {0, 607, 466, 485, 582, 485, 540};
int16_t values[3] = {0, 0, 0};

/**
 * Object Declarations
 */
MeDCMotor leftMotor(M1); // Forward is -255
MeDCMotor rightMotor(M2); // Forward is 255
MeLineFollower lineFinder(PORT_4);
MeRGBLed led(0,30);
MeBuzzer buzzer;

/**
 * Set both of mBot LEDs to a certain colour
 *
 * @param[in] r This is the red value for the desired colour
 * @param[in] g This is the green value for the desired colour
 * @param[in] b This is the blue value for the desired colour
 */
void mbot_led(int16_t r = 0, int16_t g = 0, int16_t b = 0) {
    led.setColor(r, g, b);
    led.show();
}

/**
 * Toggle the selected LED on the Colour Sensor LED based on the ID chosen
 *
 * @param[in] colour Colour Sensor LED ID
 */
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

/**
 * Toggle the selected LED on the Colour Sensor LED based on the ID chosen
 *
 * @return The distance (in cm; floating) between the Ultrasonic Sensor and the wall
 */
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

/**
 * Set motor speed for each motor (left and right)
 *
 * @param[in] left Left motor speed
 * @param[in] right Right motor speed
 * @param[in] wait Set delay after assigning motor speed (in ms; default is 0ms)
 */
void move(int16_t left, int16_t right, int16_t wait = 0) {
    leftMotor.run(-left);
    rightMotor.run(right);
    delay(wait);
}

/**
 * PID movement using ultrasonic sensor
 * Formulas:
 * e(t) = (target - current) / target
 * E(t) = K_{P} * e(t) + K_{D} * d/dt (e(t))
 */
void PID_ultra() {
  float left = 255;
  float right = 255;
  float curr_distance = ultra_read();
  if (curr_distance != 0) {
    // if robot is too near the wall, error is +ve, else -ve
    float error = (ULTRA_TARGET - curr_distance) / ULTRA_TARGET; // e(t)
    float output = kP_ULTRA * error + kD_ULTRA * (error - prev_error_ultra); // E(t)
    left = 255 * (1 + output);
    right = 255 * (1 - output); 
    prev_error_ultra = error; //updating previous error
  }
  move(left, right);
}

/**
 * PID movement using IR proximity sensor circuit
 * Formulas:
 * e(t) = (target - current) / target
 * E(t) = K_{P} * e(t) + K_{D} * d/dt (e(t))
 */
void PID_IR() {
  float left = 255;
  float right = 255;
  float curr_distance = analogRead(IR);
  if (curr_distance != 0) {
    // if robot is too near the wall, error is +ve, else -ve
    float error = (IR_TARGET - curr_distance) / IR_TARGET; // e(t)
    float output = kP_IR * error + kD_IR * (error - prev_error_IR); // E(t)
    // set motor speed
    left = 255 * (1 - output);
    right = 255 * (1 + output); 
    prev_error_IR = error; //updating previous error
  }
  move(left, right);
}

/**
 * Calculates the average reading out of n measured readings using the LDR in the colour sensor
 *
 * @param[in] times The number of measured readings, n
 * @return The average reading calculated
 */
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

/**
 * Measures the coloured tile that is below the colour sensor
 * Euclidean distance = sqrt((diff. in r_value)^2 + (diff. in g_value)^2 + (diff. in b_value)^2)
 *
 * @return The colour ID of the measured colour
 */
int16_t check_colour() {
  //Measures LDR for each RGB component by flashing each of the coloured LEDS one at a time and saving the corresponding LDR value
  for (int16_t i = 0; i < 3; i += 1) {
    toggle_led(i);
    delay(RGB_WAIT);
    values[i] = getAvgReading(5);
    delay(RGB_WAIT);
  }
  toggle_led(OFF_LED);  
  double min_distance;
  int16_t colour = BLACK;
  //k-NN alorithm implementation to find the closest sample
  for (int16_t i = 0; i < 7; i += 1) {
      double distance = 0;
      distance += fabs(r_values[i] - values[0]) * fabs(r_values[i] - values[0]);
      distance += fabs(g_values[i] - values[1]) * fabs(g_values[i] - values[1]);
      distance += fabs(b_values[i] - values[2]) * fabs(b_values[i] - values[2]);
      distance = sqrt(distance); //euclidean distance
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

/**
 * Overall Movement Algorithm 
 */
void overall_movement() {
  float ultra = ultra_read();
  mbot_led();
  if (ultra > 0.00 && ultra <= ULTRA_LOWER_RANGE) {
    //beyond ULTRA_LOWER_RANGE; i.e. closer to the left
    mbot_led(255, 0, 0);
    PID_ultra();
  }
  else if (ultra >= ULTRA_UPPER_RANGE && analogRead(IR) <= IR_LIMIT) {
    //beyond ULTRA_UPPER_RANGE; i.e. closer to the right
    mbot_led(0, 0, 255);
    PID_IR();
  }
  else {
    //between ULTRA_LOWER_RANGE and ULTRA_UPPER_RANGE || ultra > 25
    mbot_led(0, 255, 0);
    move(255, 255);
  }
}

/**
 * Buzzer Tone played at the end
 * Snippet of the song "Never Gonna Give You Up" by Rick Astley
 */
void song() {
  buzzer.tone(294, 125 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(392, 125 / 2);
  buzzer.tone(330, 125 / 2);

  buzzer.tone(440, 125 * 3 / 2);
  buzzer.tone(440, 125 * 3 / 2);
  buzzer.tone(392, 125 * 3 / 2);
  buzzer.tone(370, 125 / 2);
  buzzer.tone(330, 125);
  buzzer.tone(294, 125 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(392, 125 / 2);
  buzzer.tone(330, 125 / 2);

  buzzer.tone(392, 250);
  buzzer.tone(440, 125);
  buzzer.tone(370, 125 * 3 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(294, 125);
  buzzer.tone(294, 125);
  buzzer.tone(294, 125);

  buzzer.tone(440, 250);
  buzzer.tone(392, 500);
  buzzer.tone(294, 125 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(392, 125 / 2);
  buzzer.tone(330, 125 / 2);

  buzzer.tone(494, 125 * 3 / 2);
  buzzer.tone(494, 125 * 3 / 2);
  buzzer.tone(440, 375);
  buzzer.tone(294, 125 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(392, 125 / 2);
  buzzer.tone(330, 125 / 2);

  buzzer.tone(587, 250);
  buzzer.tone(370, 125);
  buzzer.tone(392, 125 * 3 / 2);
  buzzer.tone(370, 125 / 2);
  buzzer.tone(330, 125);
  buzzer.tone(294, 125 / 2);
  buzzer.tone(330, 125 / 2);
  buzzer.tone(392, 125 / 2);
  buzzer.tone(330, 125 / 2);

	buzzer.tone(392, 250);
	buzzer.tone(440, 125);
	buzzer.tone(370, 125 * 3 / 2);
	buzzer.tone(330, 125 / 2);
	buzzer.tone(294, 250);
	buzzer.tone(294, 125);
	buzzer.tone(440, 250);
	buzzer.tone(392, 500);
  buzzer.noTone();
}

/**
 * setup()
 * set pinMode for all I/O pins, disable motors and all LEDs
 * Wait for button press to star the robot (button is found on-board on the mCore)
 */
void setup() {
  led.setpin(13);
  pinMode(IR, INPUT);
  pinMode(LDR, INPUT);
  pinMode(A7, INPUT);
  pinMode(DECODER_A, OUTPUT);
  pinMode(DECODER_B, OUTPUT);
  toggle_led(OFF_LED);
  mbot_led();
  move(0, 0);
  while(analogRead(A7) > 100){} // wait for button press
}

/**
 * loop()
 * If the robot senses a line, it stops and checks the colour,
 * followed by displaying the detected colour,
 * and moving based according to the specified action given by that colour.
 * Else, it moves based on the overall movement algorithm.
 */
void loop() {
  if (lineFinder.readSensors() == S1_IN_S2_IN) {
    move(0, 0);
    mbot_led();
    switch(check_colour()) {
      case WHITE:
        // Play Song
        mbot_led(255, 255, 255);
        song();
        break;
      case RED:
        // Turn Left
        mbot_led(255, 0, 0); 
        move(-255, 255, TURNING_TIME + 10);
        move(0, 0, MOVE_WAIT);
        break;
      case GREEN:
        // Turn Right
        mbot_led(0, 255, 0); 
        move(255, -255, TURNING_TIME + 30);
        move(0, 0, MOVE_WAIT);
        break;
      case BLUE:
        // Two successive right-turns in two grids
        mbot_led(0, 0, 255); 
        move(255, -255, TURNING_TIME + 35);
        move(0, 0, MOVE_WAIT);
        move(255, 255, FORWARD_TIME + 10);
        move(255, -255, TURNING_TIME + 100);
        move(0, 0, MOVE_WAIT);
        break;
      case ORANGE:
        // 180° turn within the same grid
        mbot_led(255, 165, 0); 
        move(-255, 255, TURNING_TIME * 2 + 50);
        move(0, 0, MOVE_WAIT);
        break;
      case PURPLE:
        // Two successive left-turns in two grids
        mbot_led(128, 0, 128); 
        move(-255, 255, TURNING_TIME + 30);
        move(0, 0, MOVE_WAIT);
        move(255, 255, FORWARD_TIME - 80);
        move(-255, 255, TURNING_TIME + 85);
        move(0, 0, MOVE_WAIT);
        break;
    }
    mbot_led();
  }
  else {
    overall_movement();
  }
  delay(10); // a delay of 10ms as the robot has unexplained behaviour when the loop time is <10ms
}
