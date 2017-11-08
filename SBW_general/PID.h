#ifndef PID_h
#define PID_h

#include <SPI.h>
#include <math.h>

/*Define definite variables*/
//PID
#define PWM_front 8
#define DIR 46
#define K_p 200 //200
#define K_d -1.5 // -1.5 ish
#define K_i 0

/*Define functions*/
//PID
void PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int, float, float);


#endif //PID_h

//Gameplan:
//Try out different gains off ground for holding position (stationary testing)
//Try out different gains ON ground (max 45 minutes) (step input)
//if failed, try different gains OFF ground, tune (step input)
