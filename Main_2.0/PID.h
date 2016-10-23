#ifndef PID_h
#define PID_h

#include <SPI.h>
#include <math.h>

/*Define definite variables*/
//PID
#define PWM_front 9
#define K_p 190
#define K_d 8
#define K_i 0
#define DIR 46

/*Define functions*/
//PID
void PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int);


#endif //PID_h
