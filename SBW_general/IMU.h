#ifndef IMU_h
#define IMU_h

#include <SPI.h>
#include <math.h>

/*Define definite variables*/
//IMU
#define CSN 65
#define SO 74
#define SI 75
#define CLK 76

/*Define functions*/
//Initialize IMU
void initIMU(void);

//IMU data
float getIMU(byte);


#endif //IMU_h
