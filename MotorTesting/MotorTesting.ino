#include "IMU.h"
#include "PID.h"
#include <math.h>

//Front motor
#define PWM_front 8 //PWM pin for front motor, controls front wheel turning speed
#define DIR 11 //Direction pin of the front motor
#define RunStop 46 //HIGH runs the motor, LOW stops
int steer_dir = 0; //steer direction

float percent_of_max_speed; //value between 0 and 1

//define maximum front wheel pwm
float max_front_pwm = 200;
float min_front_pwm = 80; //if deadband, we can turn up the min pwm


void setup() {
  Serial.begin(115200);//PC baud rate
  analogWrite(RunStop,0); //set motor to run - low
  analogWrite(DIR,0);//255 is clockwise, 0 is CCW
}
//MAIN LOOP

void loop() {
  float raw_speed = .01; //value between 0 and 1 //% of max seed
  int mapped_PWM = map(raw_speed, 0, 1, min_front_pwm, max_front_pwm);
  //mapped maps raw_speed between 0 and 1 to be between 0 and 200
  //75 PWM means motor does not move. 200 PWM gives 4V to motor which is max spped
  analogWrite(PWM_front, mapped_PWM); //

  analogWrite(DIR,255);
  delay(1000);
  //analogWrite(DIR,255);
  delay(1000);
}


