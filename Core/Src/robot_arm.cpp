#include "robot_arm.h"

Robot_Arm::Robot_Arm()
{
    for (int i = 0; i < 3; i++)
        init_angle[i] = 0;
}

Robot_Arm::Robot_Arm(TIM_HandleTypeDef *t, const int i_a[3])
{
    motor[0].set(t, TIM_CHANNEL_1);
    motor[1].set(t, TIM_CHANNEL_2);
    motor[2].set(t, TIM_CHANNEL_3);
    this->move(init_angle);
}

Robot_Arm::~Robot_Arm()
{
    this->move(init_angle);
}

void Robot_Arm::set(TIM_HandleTypeDef *t, const int i_a[3])
{
    motor[0].set(t, TIM_CHANNEL_1);
    motor[1].set(t, TIM_CHANNEL_2);
    motor[2].set(t, TIM_CHANNEL_3);
    this->move(init_angle);
}

void Robot_Arm::move(const int angle[3])
{
    for (int i = 0; i < 3; i++)
        motor->output(angle[i] * 100 / 9 + 1500);
}