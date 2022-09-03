#include "robot_arm.h"

Robot_Arm::Robot_Arm()
{
    for (int i = 0; i < 3; i++)
        init_angle[i] = 0;
}

Robot_Arm::Robot_Arm(TIM_HandleTypeDef *t, const int i_a[3])
{
    for (int i = 0; i < 3; i++)
        init_angle[i] = 0;
    this->set(t, i_a);
}

Robot_Arm::~Robot_Arm()
{
    this->move(init_angle);
}

/**
 * @brief Set timer and channel
 * @param t TIM handle
 * @param  i_a initial angle of joints
 * @retval none
 */ 
void Robot_Arm::set(TIM_HandleTypeDef *t, const int i_a[3])
{
    motor[0].set(t, TIM_CHANNEL_1);
    motor[1].set(t, TIM_CHANNEL_2);
    motor[2].set(t, TIM_CHANNEL_3);
    this->move(init_angle);
}

/**
 * @brief a function to move joint to desired angle
 * @param angle angle in degree from -90 to 90
 * @retval none
 */
void Robot_Arm::move(const int angle[3])
{
    for (int i = 0; i < 3; i++)
        motor[i].output(angle[i] * 100 / 9 + 1500);
}