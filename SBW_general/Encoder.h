#ifndef Encoder_h
#define Encoder_h

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

#endif //I_h

