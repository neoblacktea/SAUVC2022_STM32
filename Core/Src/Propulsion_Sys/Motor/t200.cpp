#include "Propulsion_Sys/Motor/t200.h"

T200::T200()
{
    timer = nullptr;
}

T200::T200(TIM_HandleTypeDef* t, uint32_t c)
{
    timer = t;
    channel = c;
    __HAL_TIM_SetCompare(timer, channel, 1500);
    HAL_TIM_PWM_Start(timer, channel);
}

T200::~T200()
{
    __HAL_TIM_SetCompare(timer, channel, 1500);
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
void T200::set(TIM_HandleTypeDef* t, uint32_t c)
{
    timer = t;
    channel = c;
    __HAL_TIM_SetCompare(timer, channel, 1500);
    HAL_TIM_PWM_Start(timer, channel);
}

/**
 * @brief Transform force value to PWM signal then output
 * @param force force in kgf
 * @retval none
 */ 
void T200::output(float force)
{
    if (timer == NULL)
        return;
    int signal = 1500;
    int high = 200;
    int low = 0;
    int mid = 100;

    //use binary search to find the closest region 
    while (low < high)
    {
        mid = (high + low) / 2;
        if (force > FORCE[mid])
            low = mid + 1;
        else if (force < FORCE[mid])
            high = mid - 1;
        else
            break;
    }
    //find the closet value
    if (fabs(force - FORCE[mid]) > fabs(force - FORCE[mid - 1]))
        signal = SIGNAL[mid - 1];
    else if (fabs(force - FORCE[mid]) > fabs(force - FORCE[mid + 1]))
        signal = SIGNAL[mid + 1];
    else
        signal = SIGNAL[mid];
    //output PWM signal
    __HAL_TIM_SetCompare(timer, channel, signal);
}