#include "encoders.h"
#include "PID.h"
#include "kinematics.h"
#include "lineSensors.h"
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
float prox_meas;
float dist_als;
#define SCALING 3


PID pidLeft( kpReturn, kiReturn, kdReturn );
PID pidRight( kpReturn, kiReturn, kdReturn );

Kinematics kinematics;
unsigned long speed_update_ts;
unsigned long demand_update_ts;
long countLeft_prev;
long countRight_prev;

unsigned spd_update_ts;
unsigned long pid_update_ts;
unsigned long dir_switch_ts;
float dir;
float pwr;
float pwr_inc;
float demand;
float left_speed_loop;
float right_speed_loop;
float last_error_left;
float last_error_right;
float integral_error_left;
float integral_error_right;
unsigned long pid_update_dt;
unsigned long start_time;

int leftSensorRead,  centreSensorRead,  rightSensorRead; //define sensor readings

lineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

float pwr_left;
float pwr_right;


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

  line_left.calib();
  line_centre.calib();
  line_right.calib();

  leftSensorRead=0;
  centreSensorRead=0;
  rightSensorRead=0;

  speed_update_ts=millis();

  demand_update_ts=millis();

  countLeft_prev= countLeft;
  countRight_prev= countRight;
  // Initialise the Serial communication

  delay(1000);
  dir_switch_ts=millis();
  dir=1;
  pwr=0;
  pwr_inc=10;
  demand=1.5f;
  float left_speed_loop=0;
  float right_speed_loop=0;
  integral_error_left=0;
  integral_error_right=0;

  
}
 

// Remember, loop runs again and again
void loop(){

 
  straightLine();    
  
}

void straightLine() {
  


  //Testing stop change to demand
  /*
  if(millis() -demand_update_ts >1000) {
    demand_update_ts=millis();
    demand *= -1;
  }
  */

  //Verify if everything is working
  /*
  leftMotor(20.0f);
  Serial.print(countLeft);
  Serial.print('\n');
  Measure speed, verify
  */

  //Measure if the speed component works for both wheels -> works
  //Measure the control relationship of both wheels
  unsigned long speed_update_dt;

  speed_update_dt= millis()-speed_update_ts;
  if(speed_update_dt>50) {
    speed_update_ts= millis();
    
    long left_diff;
    long right_diff;
  
    
    left_diff= countLeft - countLeft_prev;
    left_speed_loop= (float)left_diff;
    right_diff= countRight - countRight_prev;
    right_speed_loop= (float)right_diff;

    countLeft_prev = countLeft;
    countRight_prev = countRight;
    
    //velocity of wheel in counts/s
    left_speed_loop= left_speed_loop/speed_update_dt;
    right_speed_loop= right_speed_loop/speed_update_dt;
    //update last count

  } 

  //Serial.print(',');
  //Serial.print(right_speed_loop);




  
  if(millis() - dir_switch_ts>3000) {
    dir_switch_ts= millis();
    //dir=dir*-1;
    demand=demand*dir;
  }
  
  pid_update_dt = millis() - pid_update_ts;
  if(pid_update_dt>50) {
     pid_update_ts= millis();
  // Determine error, verify
    
    float error_left = demand - left_speed_loop;
    float error_right = demand - right_speed_loop;

    
    //Derivative
    float d_error_left= (error_left-last_error_left)/ (float) pid_update_dt;
    float d_error_right= (error_right-last_error_right)/(float) pid_update_dt;

    last_error_left=error_left;
    last_error_right=error_right;

    //Integral 
    integral_error_left= integral_error_left+ (error_left*(float) pid_update_dt);
    integral_error_right= integral_error_right+ (error_right*(float) pid_update_dt);
        
    float kp=40;
    float p_term_left= error_left*kp;
    float p_term_right= error_right*kp;
    
    float kd=2;
    float d_term_left = d_error_left*-kd;
    float d_term_right = d_error_right*-kd;

    float ki=0.00983;
    float i_term_left= integral_error_left*ki;
    float i_term_right= integral_error_right*ki;

    pwr_left=p_term_left+d_term_left+i_term_left;
    pwr_right=p_term_right+d_term_right+i_term_right;
    /*
    Serial.print(left_speed_loop);
    Serial.print(',');
    Serial.print(error_left);
    Serial.print(',');
    Serial.print(demand);
    Serial.print(',');
    Serial.print(d_term_left);
    //Serial.print(',');
    //Serial.print(i_term_left);
    Serial.print('\n');
    */
    /*
    Serial.print(right_speed_loop);
    Serial.print(',');
    Serial.print(error_right);
    Serial.print(',');
    Serial.print(demand);
    Serial.print(',');
    Serial.print(d_error_right);
    Serial.print('\n');
    */
  }

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



  light_meas= sensor.readAmbientSingle();
  prox_meas= sensor.readRangeSingleMillimeters();
  dist_als= 1000*sqrt(80/(2*PI*light_meas));
  unsigned long elapsed_duration= millis()-start_time;

  Serial.print(prox_meas);  
  Serial.print(',');
  Serial.print(dist_als);
  Serial.print(',');
  Serial.print(light_meas);
  Serial.print(',');  
  Serial.print(elapsed_duration/1000);
  Serial.print('\n');
  

   leftMotor(pwr_left+turn_pwm);
   rightMotor(pwr_right-turn_pwm);
  
  /*
  leftMotor(pidLeft.update(demand, left_speed_loop));
  rightMotor(pidRight.update(demand, right_speed_loop));
  */
}

// Defining motor functions
void leftMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {

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

  }
  else {
    if (speed >= 0) digitalWrite( R_DIR_PIN, LOW );
    else digitalWrite( R_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( R_PWM_PIN, speed );
  }
}
