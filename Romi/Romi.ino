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
#define kpReturn 3.50
#define kiReturn 0.00025
#define kdReturn 0.05
#define kpLineFF 5.25
#define kiLineFF 0.05
#define kdLineFF 0.15

#define BUTTON_A 14
#define LED 13

#define STATE_INITIAL  0
#define STATE_DO_EXP   1
int wasItPressed = 0;
int state = 0;

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


PID pidLeft( kpReturn, kiReturn, kdReturn );
PID pidRight( kpReturn, kiReturn, kdReturn );

lineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics kinematics;


void setup() 
{
  // Initialise your other globals variables
  // and devices.

  pinMode(LED, OUTPUT);
  pinMode(BUTTON_A, INPUT);

  setupEncoder0();
  setupEncoder1();

  line_left.calib();
  line_centre.calib();
  line_right.calib();


  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);

  speed_update_ts=millis();
  demand= 7.0f;
  demand_update_ts=millis();

  countLeft_prev= countLeft;
  countRight_prev= countRight;

  leftSensorRead = 0;
  centreSensorRead = 0;
  rightSensorRead = 0;

  wasItPressed = 0;
  state = STATE_INITIAL;

  
  // Initialise the Serial communication
  Serial.begin(9600);
  // Initilizing beeps
  PlayBeep(4, 100); PlayBeep(4, 100); PlayBeep(4, 100);
  delay(1000);
  Serial.println("***RESET***");
}
 

// Remember, loop runs again and again
// once reset, wasItPressed is reset to 0
void loop(){
    // HIGH -> button released, LOW -> button presssed
    
    if( state == STATE_INITIAL ) {

        //listen for button press, if pressed change state
        int buttonState = digitalRead(BUTTON_A);
        if (buttonState == LOW) {
          state = STATE_DO_EXP;
          wasItPressed = 1;
        }

    }
    else if ( state == STATE_DO_EXP ) {
        experiment();
    }
    else {
        Serial.print("System Error, Unknown state: ");
        Serial.println( state );
    }
}




// the actual experiment code :)
void experiment(){
      light_meas = sensor.readAmbientSingle();
      float theta_error = kinematics.getTheta();

      // write out some stats
      Serial.print("speed_here");
      Serial.print(',');
      Serial.print("dist_here");
      Serial.print(',');
      Serial.println(light_meas);


      if(light_meas<1200) {
        go_straight();
      }
      else { //stop
        leftMotor(0.0f);
        rightMotor(0.0f);
        PlayBeep(4,100); PlayBeep(4,100);

        // change state to indicate end of experiment
        state = STATE_INITIAL;
       
      } 

       // other code
      //  // its getting close to light source, so adjust speed
      //  if (new_light_measure > old_light_measure ){
      //    // this means that romi is going towards a light source/obstacle
      //    // we want to slow down
      //    new_speed = current_speed - 10;
      //    leftMotor(new_speed);
      //    rightMotor(new_speed);
      //  }
      //  else {
      //    // ...
      //    new_speed = current_speed + 10;
      //    leftMotor(new_speed);
      //    rightMotor(new_speed);
      //  }
}


void go_straight() {
    float theta_error = kinematics.getTheta();
    int turn_pwm = 0;

    leftSensorRead = line_left.readCalib();
    centreSensorRead = line_centre.readCalib();
    rightSensorRead = line_right.readCalib();

    bool left_on_line=false;
    bool right_on_line=false;

    if (leftSensorRead > 150) left_on_line = true;
    if (rightSensorRead > 150) right_on_line = true;
    
    float left_speed= pidLeft.update(demand, left_speed_loop);
    float right_speed= pidRight.update(demand, right_speed_loop);

    if (left_on_line==false) {
      turn_pwm = 2.0;
    }
    else if (right_on_line==false) {
      turn_pwm = -2.0;
    }
    else if (left_on_line==true && right_on_line==true) {
      turn_pwm = 0;
    }
  
    int left_demand  =  left_speed + turn_pwm;
    int right_demand =  right_speed - turn_pwm;
    
    leftMotor(left_demand);
    rightMotor(right_demand);
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


void PlayBeep(int volume, int delay_ms){
  analogWrite(BUZZER, volume);
  delay(delay_ms);
  analogWrite(BUZZER, 0);
  delay(delay_ms);
}
