//============================================ IO Pins ============================================
// Pins used on Arduino Uno

#define lpwm D8
#define ldir1 D7
#define ldir2 D6

#define rpwm D5
#define rdir1 D4
#define rdir2 D3

int i=0;

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis fabo_9axis;

// Left and Right Initial start speeds
int rightSpeed = 1023;
int leftSpeed = 1023;

//PID Values
float inputx, inputy, inputz, output;
float error, lasterror=0;
float integral=0;
float proportional, derivative;

// pidcount is used to divide the total error (integral formula)
int pidcount = 1;

// PID Multipliers
float kp = 0.8;
float ki = 0;
float kd = 0.2;

// The setpoint is used in the PID equation
float setPointx = 0;
//float setpointy = refvaly;

//============================================= Setup =============================================
void setup(){
 
  Serial.begin(9600);
 
//  pinMode(rEncoder, INPUT);
// pinMode(lEncoder, INPUT);

    fabo_9axis.begin();
    pinMode(lpwm, OUTPUT);
  pinMode(ldir1, OUTPUT);
  pinMode(ldir2, OUTPUT);
  
  pinMode(rpwm, OUTPUT);
  pinMode(rdir1, OUTPUT);
  pinMode(rdir2, OUTPUT);

  digitalWrite(ldir2, LOW);
  digitalWrite(ldir1, HIGH);
  digitalWrite(rdir1, LOW);
  digitalWrite(rdir2, HIGH);
//  analogWrite(lpwm, 1023);
//  analogWrite(rpwm, 1023);
}// End Setup

//============================================= Loop ==============================================
void loop(){

  digitalWrite(ldir2, LOW);
  digitalWrite(ldir1, HIGH);
  digitalWrite(rdir1, LOW);
  digitalWrite(rdir2, HIGH);

  fabo_9axis.readMagnetXYZ(&inputx,&inputy,&inputz);

  //========== Right ==========
  
  // Serial.println(input);
  // Calculate the PID values
  proportional = setPointx - inputx;
  derivative = proportional - lasterror;
  integral = (integral + proportional)/pidcount;
 
  // Scale the PID values and save total as output
  output = kp * proportional + kd * derivative + ki * integral;
  Serial.println(output);
 
  // Save variables for next time
  lasterror = proportional;
 
  rightSpeed = -output + 300;
  rightSpeed = constrain(rightSpeed,250,350);

  //========== Left ==========
  leftSpeed = 300+output;
  leftSpeed = constrain(leftSpeed,250,350);
  pidcount++;
 
  // Finally, set the updated value as new speed
  
  analogWrite(rpwm, rightSpeed);
  analogWrite(lpwm, leftSpeed);

 
  delay(100);

//  if(i<10)
//  {
//    i++;
//  }
//  else
//  {
//    i=0;
//    digitalWrite(ldir2, LOW);
//    digitalWrite(ldir1, LOW);
//    digitalWrite(rdir2, LOW);
//    digitalWrite(rdir1, LOW);
//    analogWrite(rpwm, 0);
//    analogWrite(lpwm, 0);
//    delay(1000);
//  }
} // End Loop
 
//============================================== PID ==============================================

// Space to add whats in the loop later on



