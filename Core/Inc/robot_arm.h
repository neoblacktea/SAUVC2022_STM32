#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "motor.h"

class Robot_Arm
{
private:
    Motor motor[3];
    int init_angle[3];

public:
    Robot_Arm();
    Robot_Arm(TIM_HandleTypeDef *t, const int i_a[3]);
    ~Robot_Arm();
    void set(TIM_HandleTypeDef *t, const int i_a[3]);
    void move(const int angle[3]);
};

#endif