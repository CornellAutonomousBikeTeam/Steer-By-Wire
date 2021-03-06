#include "IMU.h"
#include "PID.h"
#include <math.h>

bool IS_BALANCE_CONTROLLER_ON = false;
bool DOES_HANDLEBAR_INPUT_TO_BALANCE_CONTROLLER = true;


//Timed Loop Variables
const long interval = 10000;
long l_start;
long l_diff;
float tNew = 0;

//Front motor
#define PWM_front 8 //PWM pin for front motor, controls front wheel turning speed
#define DIR 46 //Direction pin of the front motor   11
//#define RunStop 46 //HIGH runs the motor, LOW stops    46
int steer_dir = 0; //steer direction

//Balance control constants
const int k1 = 95; //71; //lean    71
const int k2 = 35; //10; // lean rate   10 
const int k3 = -40; // steer

//define maximum front wheel pwm
float min_front_PWM = 8;
float max_front_PWM = 200;    // 

//Read the relative position of the encoders
signed int relativePosWheel = REG_TC0_CV0;
signed int relativePosHandle = REG_TC2_CV0;

//Read the index value (Z channel) of the encoders
signed int indexValueWheel = REG_TC0_CV1;
signed int indexValueHandle = REG_TC2_CV1;

//Encoder 1 - Front Wheel (variables denoted by '_W')

const int quad_A_W = 2;
const int quad_B_W = 13;
const int idx_W = 60; 
const unsigned int mask_quad_A_W = digitalPinToBitMask(quad_A_W);
const unsigned int mask_quad_B_W = digitalPinToBitMask(quad_B_W); 
const unsigned int mask_idx_W = digitalPinToBitMask(idx_W); 
int REnot = 3;
int DE = 6;
signed int oldPosition_W  = 0;
signed int oldIndex_W = 0;
unsigned long previous_t = 0;
signed int x_offset_W = 0;
float desired_pos_W = 0;
float current_pos_W = 0;
float current_vel = 0;
float desired_vel = 0;
float vel_error = 0;
float pos_error = 0;
float PID_output = 0;
float sp_error = 0;
float sv_error = 0;
int pwm = 0;


//Encoder 2 - Handlebar (variables denoted by '_H')

const int quad_A_H = 4;
const int quad_B_H = 5;
const int idx_H = 10;  // 10 - C , 77 - A 
const unsigned int mask_quad_A_H = digitalPinToBitMask(quad_A_H);
const unsigned int mask_quad_B_H = digitalPinToBitMask(quad_B_H); 
const unsigned int mask_idx_H = digitalPinToBitMask(idx_H); 
signed int oldPosition_H  = 0;
signed int oldIndex_H = 0;
unsigned long previous_t_H = 0;
signed int x_offset_H = 0;
float current_pos_H = 0;


//count the number of times the time step has been calculated to calculate a running average time step
//int numTimeSteps = 0;
//float averageTimeStep = 0;
//int n = 0;

float desired_steer = 0;
//float desired_pos_array[250];
//float theo_position = 0;
//
//#define relay3 50
//#define relay4 49

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//PC baud rate
  initIMU();
   Serial.println("Start"); //Trigger Word for Raspberry Pi to know a test is starting
   Serial.println("K1="+String(k1)+" K2="+String(k2)+" K3="+String(k3)); //print to csv in pi
   Serial.println(String("millis")+", "+"encoder_position_H"+", "+"encoder_position_W"+
                ", "+ "imudata.angle"+ ", " + "imudata.rate" + ", " + "desired_pos_W"); //print to csv in pi
  
  // ---- setup up DUE's clock and quadrature decoder ---- //
  
  // activate peripheral functions for quad pins
  
  REG_PIOB_PDR = mask_quad_A_W;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A_W;   // choose peripheral option B    
  REG_PIOB_PDR = mask_quad_B_W;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B_W;   // choose peripheral option B 
  REG_PIOB_PDR = mask_idx_W;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx_W;   // choose peripheral option B 

  REG_PIOC_PDR = mask_quad_A_H;     // activate peripheral function (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_quad_A_H;   // choose peripheral option C    
  REG_PIOC_PDR = mask_quad_B_H;     // activate peripheral function (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_quad_B_H;   // choose peripheral option C 
  REG_PIOC_PDR = mask_idx_H;     // activate peripheral function (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_idx_H;   // choose peripheral option C 


// activate clock for TC0 and TC1 and TC2
  REG_PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31; REG_PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35; 

      // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;
  REG_TC2_CMR0 = 5; 
  
    // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12);
  REG_TC2_BMR = (1<<9)|(1<<8)|(1<<12);
  
    // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
  REG_TC0_QIER = 1;
  REG_TC2_QIER = 1;
  
    // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
    // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;
  REG_TC2_CCR0 = 5; 

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
//  pinMode (RunStop, OUTPUT);
  analogWrite(PWM_front, 0);
//  digitalWrite(RunStop, LOW);
  digitalWrite(DIR, HIGH);

//Calibration (TICKING)

  signed int y = REG_TC0_CV1; 
  oldIndex_W = y;
  digitalWrite(DIR, HIGH); 
  while(y==oldIndex_W){
    //analogWrite(PWM_front, 100);
    y = REG_TC0_CV1;
    //Serial.println("Ticking 1");
  }
  
  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
   x_offset_W = REG_TC0_CV0;

  /*
  signed int y2 = REG_TC2_CV1; 
  oldIndex_H = y2;
  digitalWrite(DIR, HIGH); 
  while(y2==oldIndex_H){
    //analogWrite(PWM_front, 100);
    y2 = REG_TC2_CV1;
    Serial.println(y2);
    //Serial.println("Ticking 2");
  }

  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  x_offset_H = REG_TC2_CV0;
*/

  Serial.println("Setup done");
}//setup end

// other functions
/* intakes commanded velocity from balance controller
 * converts commanded velocity into commanded position */
float eulerIntegrate(float desiredVelocity, float current_pos_W){
  float desiredPosition = current_pos_W + desiredVelocity * ((float)interval/1000000.0) ;
  //lefloat desiredPosition = 0;
  return desiredPosition;
}


//-----
// updates global variables representing Front Wheel encoder position 
float updateEncoderPositionWheel(){
  //Read the relative position of the encoder
  relativePosWheel = REG_TC0_CV0;
  //Read the index value (Z channel) of the encoder
  indexValueWheel = REG_TC0_CV1;
  current_pos_W = (((relativePosWheel - x_offset_W) * 0.02197 * M_PI)/180); //Angle (rad)
  return current_pos_W;
}
// updates global variables representing Handlebar encoder position 
float updateEncoderPositionHandle(){
  //Read the relative position of the encoder
  relativePosHandle = REG_TC2_CV0;
  //Read the index value (Z channel) of the encoder
  indexValueHandle = REG_TC2_CV1;
  current_pos_H = (((relativePosHandle - x_offset_H) * 0.02197 * M_PI)/180); //Angle (rad)
  return current_pos_H;
}
//----


/* takes in desired position and applies a PID controller to minimize error between current position and desired position */
void frontWheelControl(float desiredVelocity, float current_pos_W){  //steer contribution doese not need to be passed into 
  //Serial.println("Entered Fucntion");                                                //frontWheelControl because it is a global variable 
  unsigned long current_t = micros();
  if (IS_BALANCE_CONTROLLER_ON){
    desired_pos_W = eulerIntegrate(desiredVelocity, current_pos_W);
  } else {
    desired_pos_W = current_pos_H;
  }
     //Serial.println(desired_pos_W);
     


  PID_Controller((desired_pos_W), relativePosWheel, x_offset_W, current_t, previous_t, oldPosition_W, max_front_PWM, min_front_PWM); 
  //inside PID.cpp, writes PWM to motor pin, added 0.11 to desired_pos to account for encoder offset
  previous_t = current_t;
  oldPosition_W = relativePosWheel - x_offset_W;
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
/*//----
  void getPeriod() {
    float tOld = tNew;
    tNew = micros();
    double T = (tNew - tOld);
    double speed = (1.2446)*(1E6) / (28 * T) ;
    if (speed < 100) {
      Serial.println(speed, 3);
    }
}*/
             

//MAIN LOOP
void loop() {
  // put your main code here, to run repeatedly:
     // l_start = micros();

   //  analogWrite(PWM_front, 8);


  //float raw_speed = .01; //value between 0 and 1 //% of max seed
  //int mapped_PWM = map(raw_speed, 0, 1, min_front_PWM, max_front_PWM);
  //maps raw_speed between 0 and 1 to be between 85 and 200
  //85 PWM means motor does not move. 200 PWM gives 4V to motor which is max spped
  //analogWrite(PWM_front, mapped_PWM); //
  
      
      float encoder_position_H = updateEncoderPositionHandle(); //output is current position wrt front zero
      if (DOES_HANDLEBAR_INPUT_TO_BALANCE_CONTROLLER){
        desired_steer= encoder_position_H; //Should we put a -1 in here for countersteering?
      } else{ 
      desired_steer =0;
      }
      //Serial.print("encoder position handlebar: "); Serial.println(encoder_position_H);
      float encoder_position_W = updateEncoderPositionWheel(); //output is current position wrt front zero
      //Serial.print("encoder position wheel: "); Serial.println(encoder_position_W);
      
      
      roll_t imu_data = updateIMUData(); //get roll angle and roll rate from IMU, stored in a struct: imu_data.roll_angle, imu_data.roll_rate
      imu_data.angle = imu_data.angle - 1.57; imu_data.rate = imu_data.rate; //correct for IMU rotation of SBW bike (90 degrees)
      //Serial.print("Roll Angle"); Serial.println(imu_data.angle);
      //analogWrite(PWM_front, 20);

      //Call the balance controller, which will return the desired velocity of the front motor - this is the velocity to turn the front wheel to 'recover' from the current roll angle/rate
          //the desired velocity will be integrated to find an angle, which is then passed into the PID_Controller function
          
      float desiredVelocity = balanceController(((1)*(imu_data.angle)),(1)*imu_data.rate, encoder_position_W); 
      
      //Integrates the desired velocity to find angle to turn, writes pwm to front motor pin using a PD controller
      //Serial.println(encoder_position_W);
      frontWheelControl(desiredVelocity, encoder_position_W);  //DESIRED VELOCITY SET TO NEGATIVE TO MATCH SIGN CONVENTION BETWEEN BALANCE CONTROLLER AND 
/*
      //Making sure loop length is not violated
      l_diff = micros() - l_start;
     // Serial.println(String(l_diff));
    //Standardize the loop time by checking if it is currently less than the constant interval, if it is, add the differnce so the total loop time is standard
     if (l_diff < interval){
        delayMicroseconds(interval - l_diff);
      }else{
        Serial.println("LOOP LENGTH WAS VIOLATED. LOOP TIME WAS: " + String(l_diff));
      }*/
              Serial.println(String(millis())+", "+ String(encoder_position_H)+", "+String(encoder_position_W)+
                ", "+ String(imu_data.angle)+ ", " + String(imu_data.rate) + ", " + String(desired_pos_W)); //send data to pi to be printed to csv
}


