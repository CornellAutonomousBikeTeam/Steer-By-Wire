#include "Encoder.h"

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
