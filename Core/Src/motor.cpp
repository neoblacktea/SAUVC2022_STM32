#include "motor.h"

Motor::Motor()
{
    timer = nullptr;
}

Motor::~Motor()
{
    this->output(1500);
}

/**
 * @brief Set timer and channel
 * @param t TIM handle
 * @param  c TIM Channels to be enabled
 *          This parameter can be one of the following values:
 *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
 *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
 *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
 *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
 */ 
void Motor::set(TIM_HandleTypeDef* t, const uint32_t c)
{
    timer = t;
    channel = c;
    __HAL_TIM_SetCompare(timer, channel, 1500);
    HAL_TIM_PWM_Start(timer, channel);
}

/**
 * @brief Output PWM signal
 * @param signal the duration of high in us
 * @retval none
 */ 
void Motor::output(const int signal)
{
    if (timer == nullptr)
        return;
    //output PWM signal
    __HAL_TIM_SetCompare(timer, channel, signal);
}