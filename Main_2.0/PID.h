#ifndef PID_h
#define PID_h

#include <SPI.h>
#include <math.h>

/*Define definite variables*/
//PID
#define PWM_front 9
#define DIR 46
#define K_p 500
#define K_d 4
#define K_i 0

/*Define functions*/
//PID
void PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int);


#endif //PID_h
