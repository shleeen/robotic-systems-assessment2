#include "encoders.h"
#include "PID.h"
#include "kinematics.h"
#include "lineSensors.h"
#include <Wire.h>
#include <VL6180X.h>
int turn_pwm = 0;

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LINE_LEFT_PIN   A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A4 //Pin for the right line sensor
#define BUZZER 6
#define kpReturn 2.0
#define kiReturn 0.00001
#define kdReturn 1.0
#define kpLineFF 5.25
#define kiLineFF 0.05
#define kdLineFF 0.15

// Proximity sensor variables
VL6180X sensor;
float light_meas;
#define SCALING 2

unsigned long speed_update_ts;
unsigned long demand_update_ts;
float left_speed_loop;
float countLeft_prev;
float right_speed_loop;
float countRight_prev;
float demand;
unsigned spd_update_ts;
int leftSensorRead,  centreSensorRead,  rightSensorRead; //define sensor readings

int left_demand;
int right_demand;
PID pidLeft( kpReturn, kiReturn, kdReturn );
PID pidRight( kpReturn, kiReturn, kdReturn );

int STATE;


lineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics kinematics;


void setup() 
{
  // Initialise your other globals variables
  // and devices.



  setupEncoder0();
  setupEncoder1();

  line_left.calib();
  line_centre.calib();
  line_right.calib();

  STATE=0;

  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);

  speed_update_ts=millis();
  demand= 1.5f;
  demand_update_ts=millis();

  countLeft_prev= countLeft;
  countRight_prev= countRight;

  leftSensorRead = 0;
  centreSensorRead = 0;
  rightSensorRead = 0;

  left_demand  =  72.0f;
  right_demand =  72.0f;
  
  // Initialise the Serial communication
  Serial.begin(9600);
  // Initilizing beeps
  PlayBeep(4, 100); PlayBeep(4, 100); PlayBeep(4, 100);
  delay(1000);
  Serial.println("***RESET***");
}
 


// Remember, loop runs again and again
void loop() {
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
  
  light_meas = sensor.readAmbientSingle();
  int turn_pwm = 0;

  leftSensorRead = line_left.readCalib();
  centreSensorRead = line_centre.readCalib();
  rightSensorRead = line_right.readCalib();

  bool left_on_line=false;
  bool right_on_line=false;

  if (leftSensorRead > 150) left_on_line = true;
  if (rightSensorRead > 150) right_on_line = true;
  //sets approximately equal speed to both motors

  if (left_on_line==false) {
    turn_pwm = 2.0;
  }
  else if (right_on_line==false) {
    turn_pwm = -2.0;
  }
  else if (left_on_line==true && right_on_line==true) {
    turn_pwm = 0;
  }
  Serial.print(light_meas);
  Serial.print(',');
  Serial.print(left_speed_loop);
  Serial.print(',');  
  Serial.print(STATE);
  Serial.print('\n');

  switch (STATE) {
    case 0:
      const_speed();
      break;

    case 1:
      red_speed();
      break;

    case 2:
      no_speed();
      break;
  }
    
}
/*
    Serial.print(left_speed_loop);
    Serial.print(',');
    Serial.print(demand);

    unsigned long pid_update_

    float error= demand - left_speed_loop;
    //P CONTROLLER      P-GAIN
    float p_cont= error*75;

    Serial.print(',');
    Serial.println(error);
    Serial.print('\n');

    leftMotor(p_cont);
*/
// GO STRAIGHT CODE //

// Sets conditions according to the threshold of red light //  
void const_speed() {
  if(light_meas<200) {
    left_demand  =  72.0f + turn_pwm;
    right_demand =  72.0f - turn_pwm;
    leftMotor(left_demand);
    rightMotor(right_demand);
    STATE=0;
  }
  else {
    STATE=1;
  }
}

// Aims to reduce "current speed" by a constant in every iteration //
void red_speed() {
   unsigned long elapsed_time2;
   elapsed_time2= millis()-speed_update_ts;
   if(light_meas>1500 && elapsed_time2>2000) {
    STATE=2;
   }
   else if(light_meas>200 && light_meas<1500) {
     if(left_speed_loop>0 ) {
        left_demand=left_demand+turn_pwm-0.85;
        right_demand= right_demand-turn_pwm-1.05;
        leftMotor(left_demand);
        rightMotor(right_demand);
        if(left_speed_loop<0 && light_meas>1500) {
          leftMotor(0);
          rightMotor(0);
          STATE=1;
        }
     
      }
   }
   else if(light_meas<200) {
      STATE=0;
    
  }


}

void no_speed() {
  unsigned long elapsed_time3;
  elapsed_time3= millis()-speed_update_ts;
  if(light_meas>1500 && elapsed_time3>2000) {
    leftMotor(0);
    rightMotor(0);
  }
  else {
    STATE=2;
  }
}

void pid() {

  light_meas = sensor.readAmbientSingle();
  int turn_pwm = 0;

  leftSensorRead = line_left.readCalib();
  centreSensorRead = line_centre.readCalib();
  rightSensorRead = line_right.readCalib();

  bool left_on_line=false;
  bool right_on_line=false;

  if (leftSensorRead > 150) left_on_line = true;
  if (rightSensorRead > 150) right_on_line = true;
  //sets approximately equal speed to both motors

  if (left_on_line==false) {
    turn_pwm = 2.0;
  }
  else if (right_on_line==false) {
    turn_pwm = -2.0;
  }
  else if (left_on_line==true && right_on_line==true) {
    turn_pwm = 0;
  }
}
  /*
  left_demand  =  42.0f + turn_pwm;
  right_demand =  47.0f - turn_pwm;

// GO STRAIGHT CODE //
  //Serial.print(left_demand);
  left_demand= left_demand - 15.0f;
  right_demand= right_demand - 15.0f;
  Serial.print(left_speed_loop);
  Serial.print('\n');


}

  */


// Aims to reduce "current speed" by 5.0f in every iteration //


// Implements a buzzer noise for checkpoint //
void PlayBeep(int volume, int delay_ms){
  analogWrite(BUZZER, volume);
  delay(delay_ms);
  analogWrite(BUZZER, 0);
  delay(delay_ms);
}
// Implements a buzzer noise for checkpoint //




// Defining motor functions //
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
// Defining motor functions //
