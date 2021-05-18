#include "encoders.h"
#include "PID.h"
#include "kinematics.h"
#include <VL6180X.h>
//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BUZZER 6

VL6180X sensor;
float light_meas;
float prox_meas;
float dist_als;
#define SCALING 3



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
int state;
int wasItPressed;
int BUTTON_A=14;


float pwr_left;
float pwr_right;

float kp=40;
float ki=0.002083;
float kd=-2;

PID left_PID(kp,ki,kd);
PID right_PID(kp,ki,kd);

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

  wasItPressed=0;
  state=0;
}
 

// Remember, loop runs again and again
void loop(){

  unsigned long speed_update_dt;

  //straightLine();

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
    Serial.println(elapsed_duration);
    Serial.print('\n');

    
   // HIGH -> button released, LOW -> button presssed
    if( state == 0 ) {
        //listen for button press, if pressed change state
        int buttonState = digitalRead(BUTTON_A);
        if (buttonState == LOW) {
            state = 1;
            wasItPressed = 1;
        }
    }
    else if ( state == 1 ) {
      if(prox_meas>100){
        leftMotor(left_PID.update(demand, left_speed_loop));
        rightMotor(right_PID.update(demand, right_speed_loop));  
 
      }
      else if(prox_meas<=100){
        leftMotor(0);
        rightMotor(0);
      }
    }
    else {
        Serial.print("System Error, Unknown state: ");
        Serial.println( state );
    }
 

  
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
    //Serial.print(error_left);
    //Serial.print(',');
    Serial.print(demand);
    //Serial.print(',');
    //Serial.print(d_term_left);
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



/*
  light_meas= sensor.readAmbientSingle();
  prox_meas= sensor.readRangeSingleMillimeters();
  dist_als= 1000*sqrt(80/(2*PI*light_meas));
*/


  

  //leftMotor(pwr_left+turn_pwm);
  //rightMotor(pwr_right-turn_pwm);
  
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
