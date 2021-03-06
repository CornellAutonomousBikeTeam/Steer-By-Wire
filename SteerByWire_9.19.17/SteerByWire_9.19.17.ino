#include "IMU.h"
#include "PID.h"
#include <math.h>

//Timed Loop Variables
const long interval = 10000;
long l_start; //loop start time?
long l_diff; // time for the loop to run
float tNew = 0;

//Front motor
#define PWM_front 8 //PWM pin for front motor, controls front wheel turning speed
#define DIR 11 //Direction pin of the front motor
#define RunStop 46 //HIGH runs the motor, LOW stops
int steer_dir = 0; //steer direction

//Balance control constants
const int k1 = 100; //lean was 71
const int k2 = 50; // lean rate was 10
const int k3 = -20; // steer was -20

//define maximum front wheel pwm
float maxfront_PWM = 80;    // motor cuts out around pwm = 136

//Read the relative position of the encoder
signed int relativePos = REG_TC0_CV0;
//Read the index value (Z channel) of the encoder
signed int indexValue = REG_TC0_CV1;

//Encoder
const int quad_A = 2;
const int quad_B = 13;
const int idx = 60; 
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B); 
const unsigned int mask_idx = digitalPinToBitMask(idx); 
int REnot = 3;
int DE = 4;
signed int oldPosition  = 0;
signed int oldIndex = 0;
unsigned long previous_t = 0;
signed int x_offset = 0;
float desired_pos = 0;
float current_pos = 0;
float current_vel = 0;
float desired_vel = 0;
float vel_error = 0;
float pos_error = 0;
float PID_output = 0;
float sp_error = 0;
float sv_error = 0;
int pwm = 0;

//Set desired steer
float desired_steer = 0; //set desired steer angle to 0

void setup() {
  Serial.begin(115200);//PC baud rate
  initIMU();

  //setup Encoder
  pinMode(REnot, OUTPUT);
  pinMode(DE, OUTPUT);
  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B    
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B 
  REG_PIOB_PDR = mask_idx;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx;   // choose peripheral option B 

    // activate clock for TC0 and TC1
  REG_PMC_PCER0 = (1<<27)|(1<<28)|(1<<29);

      // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5; 
  
    // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12);
  
    // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
  REG_TC0_QIER = 1;
  
    // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
    // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
  pinMode (RunStop, OUTPUT);
  digitalWrite(RunStop, HIGH);

//Calibration (TICKING)
  signed int y = REG_TC0_CV1; 
  oldIndex = y;
  digitalWrite(DIR, HIGH); 
  while(y==oldIndex){
    analogWrite(PWM_front,30);
    y = REG_TC0_CV1;
    Serial.println("Ticking");
  }
  
  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  x_offset = REG_TC0_CV0;
  Serial.println("Setup done");
}//setup end

// other functions
/* intakes commanded velocity from balance controller
 * converts commanded velocity into commanded position */
float eulerIntegrate(float desiredVelocity, float current_pos){
  float desiredPosition = current_pos + desiredVelocity * ((float)interval/1000000.0) ;
  return desiredPosition;
}

// updates global variables representing encoder position
float updateEncoderPosition(){
  //Read the relative position of the encoder
  relativePos = REG_TC0_CV0;
  //Read the index value (Z channel) of the encoder
  indexValue = REG_TC0_CV1;
  current_pos = (((relativePos - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
  return current_pos;
}
//----
/* takes in desired position and applies a PID controller to minimize error between current position and desired position */
void frontWheelControl(float desiredVelocity, float current_pos){  //steer contribution doese not need to be passed into 
                                                                   //frontWheelControl because it is a global variable 
  unsigned long current_t = micros();
  float desired_pos = eulerIntegrate(desiredVelocity, current_pos);
  PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition, maxfront_PWM); //inside PID.cpp, writes PWM to motor pin, 
  //removed +0.13 from desired pos (to account for encoder offset) added 0.11 to desired_pos to account for encoder offset
  previous_t = current_t;
  oldPosition = relativePos-x_offset;
}
//----
/* FUNCTION THAT RETURNS DESIRED ANGULAR VELOCITY OF FRONT WHEEL */
float balanceController(float roll_angle, float roll_rate, float encoder_angle){
  float desiredSteerRate = (k1*roll_angle) + (k2*roll_rate) + k3*(encoder_angle-desired_steer);
  return desiredSteerRate;
}
//----
struct roll_t{
  float rate;
  float angle;
};
//----// Retrieve data from IMU about roll angle and rate and return it
  struct roll_t updateIMUData(){
  roll_t roll_data;
  //get data from IMU
  float roll_angle = getIMU(0x01);   //get roll angle
  float roll_rate = getIMU(0x26);    //get roll rate
  roll_data.angle = roll_angle;
  roll_data.rate = roll_rate;
  return roll_data;
}

//MAIN LOOP
void loop() {
      l_start = micros();
      Serial.println("Loop");
 
      float encoder_position = updateEncoderPosition(); //output is current position wrt front zero
      
      roll_t imu_data = updateIMUData(); //get roll angle and roll rate from IMU, stored in a struct: imu_data.roll_angle, imu_data.roll_rate
      imu_data.angle = imu_data.angle - 1.57; imu_data.rate = imu_data.rate; //correct for IMU rotation of SBW bike (90 degrees)

      //Call the balance controller, which will return the desired velocity of the front motor - this is the velocity to turn the front wheel to 'recover' from the current roll angle/rate
      //the desired velocity will be integrated to find an angle, which is then passed into the PID_Controller function
      float desiredVelocity = balanceController(((1)*(imu_data.angle)),(1)*imu_data.rate, encoder_position);
      
      //Integrates the desired velocity to find angle to turn, writes pwm to front motor pin using a PD controller
      frontWheelControl((-1)*desiredVelocity, encoder_position);  //DESIRED VELOCITY SET TO NEGATIVE TO MATCH SIGN CONVENTION BETWEEN BALANCE CONTROLLER AND 

      //Making sure loop length is not violated
      l_diff = micros() - l_start;
      //Standardize the loop time by checking if it is currently less than the constant interval, if it is, add the differnce so the total loop time is standard
      if (l_diff < interval){
          delayMicroseconds(interval - l_diff);
      }else{
          Serial.println("LOOP LENGTH WAS VIOLATED. LOOP TIME WAS: " + String(l_diff));
      }
      
}


