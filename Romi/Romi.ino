#include "encoders.h"
#include "PID.h"
#include "kinematics.h"
#include <Wire.h>
#include <VL6180X.h>
//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LINE_LEFT_PIN   A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A4 //Pin for the right line sensor
#define BUZZER 6
#define kpReturn 70
#define kiReturn 0.0001
#define kdReturn 1
VL6180X sensor;
float light_meas;
#define SCALING 2


PID pidLeft( kpReturn, kiReturn, kdReturn );
PID pidRight( kpReturn, kiReturn, kdReturn );

Kinematics kinematics;
unsigned long speed_update_ts;
unsigned long demand_update_ts;
float left_speed_loop;
float countLeft_prev;
float right_speed_loop;
float countRight_prev;
float demand;
unsigned spd_update_ts;



void setup() 
{
  // Initialise your other globals variables
  // and devices.
  Serial.begin(9600);
  
  setupEncoder0();
  setupEncoder1();
  
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);

  speed_update_ts=millis();
  demand= 3.0f;
  demand_update_ts=millis();

  countLeft_prev= countLeft;
  countRight_prev= countRight;
  // Initialise the Serial communication

  delay(1000);
  Serial.println("***RESET***");
}
 

// Remember, loop runs again and again
void loop(){
  
  light_meas= sensor.readAmbientSingle();
  Serial.print(light_meas);
  Serial.print('\n');
  if(light_meas<1000) {
  straightLine();
  }
  else{
    leftMotor(0);
    rightMotor(0);
  }
}


void straightLine() {
  //Measure speed, verify

  //- it looks sensible, direction
  //- consider control relationship
  unsigned long elapsed_time;
  elapsed_time= millis()-speed_update_ts;
  if(elapsed_time>40) {
    speed_update_ts= millis();
    
    long left_diff;
    long right_diff;
    
    left_diff= countLeft - countLeft_prev;
    left_speed_loop= (float)left_diff;
    right_diff= countRight - countRight_prev;
    right_speed_loop= (float)right_diff;
    
    //velocity of left wheel in counts/s
    left_speed_loop= left_speed_loop/elapsed_time;
    right_speed_loop= right_speed_loop/elapsed_time;
    //update last count
    countLeft_prev = countLeft;
    countRight_prev = countRight;
  }

  leftMotor(pidLeft.update(demand, left_speed_loop));
  rightMotor(pidRight.update(demand, right_speed_loop));
}

// Defining motor functions
void leftMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Invalid Left Motor Speed.");
  }
  else {
    if (speed >= 0) digitalWrite( L_DIR_PIN, LOW );
    else digitalWrite( L_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( L_PWM_PIN, speed );
  }
}

void rightMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Motor speed is no.");
  }
  else {
    if (speed >= 0) digitalWrite( R_DIR_PIN, LOW );
    else digitalWrite( R_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( R_PWM_PIN, speed );
  }
}
