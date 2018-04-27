#include "PID.h"
#include <math.h>

void PID_Controller(float desired_pos, signed int x, signed int x_offset_W, 
  unsigned long current_t, unsigned long previous_t, signed int oldPosition_W, float maxPWM, float minPWM) {
  //when y value changes (when wheel passes index tick) print absolute position of the wheel now to see if encoder absolute position
  //is drifting
  float current_pos = (((x - x_offset_W) * 0.02197 * M_PI)/180); //Angle (rad)
 
  //write PID controller based off of error signal received from encoder
  //P term
  //calculate position error (rad)
  float pos_error = desired_pos - current_pos;

  //scaled positional error
  //position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value outputted to the motor
  float sp_error =  (K_p*pos_error);

  //D term
  //calculate velocity error
//  unsigned long current_t = micros();
  float current_vel = (((((x-x_offset_W)-oldPosition_W)*0.02197*1000000*M_PI/180.0)/(current_t-previous_t)));   //Angular Speed(rad/s)    <---- does this need to be updated?
  
  // the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). Calculated as target_velocity - current_velocity where target velocity is always 0
  //scaled velocity error
  float sv_error =  (K_d*current_vel)  ;  
  float total_error =  sp_error + sv_error ;

//Serial.print("total error: "); Serial.println(total_error);

  if (total_error <= 0) {
    analogWrite(DIR, 255); 

    total_error = constrain(total_error,-255,0);
    total_error = map(total_error, 0, -255, 2, 200); 
    analogWrite(PWM_front, (int)(total_error));
    //Serial.println(total_error);
    
  } else {
    analogWrite(DIR, 0);
    total_error = constrain(total_error,0,255);
    total_error = map(total_error, 0, 255, 2, 200); 
    analogWrite(PWM_front, (int)(total_error));
    //Serial.println(total_error);
  }

 

  //clip the maximum output to the motor by essentially saying "if the value is greater than this threshold, make the output to the motor this exact threshold value"
  oldPosition_W = x-x_offset_W;
  /*
   if (total_error > maxPWM +10 || total_error < -maxPWM - 10) {
      analogWrite(PWM_front, maxPWM);
   } 
   else{ 
      analogWrite(PWM_front, 10 + abs((int)(total_error))); 
   }
   */
}
