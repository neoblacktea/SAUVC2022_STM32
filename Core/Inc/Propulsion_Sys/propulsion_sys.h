#ifndef PROPULSION_SYS
#define PROPULSION_SYS

#include "stm32f4xx.h"
#include "Motor/t200.h"
#include "Datatype/dynamics.h"

class Propulsion_Sys
{
private:
    float thrust[8];
    T200 motor[8];
public:
    Propulsion_Sys();
    ~Propulsion_Sys();
    void set_timer(TIM_HandleTypeDef *tim1, TIM_HandleTypeDef *tim2);
    void allocate(Kinematics ctrl_input);
};

#endif