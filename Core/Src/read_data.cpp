#include "read_data.h"
#include "stm32f4xx_hal.h"
#include "usart.h"


Read_data::Read_data()
{
    index = -1;
	accessible = false;
	ch = '\n';
	size_of_data = 44; // depth + yaw + v + velocity + joint = 4+4+12+12+12 = 44
}

float Read_data::get_single_num()
{
	int i;
	float n;
	char *ch = (char *) &n;
	for(i=0;i<4;i++)
	{
		ch[i] = temp[index];
		index++;
	}
	return n;
}

void Read_data::assign_num()
{
	index = 0;
	depth = get_single_num();
	yaw = get_single_num();
	v.x = get_single_num();
	v.y = get_single_num();
	v.z = get_single_num();
	velocity[0] = get_single_num();
	velocity[1] = get_single_num();
	velocity[2] = get_single_num();
	joint[0] = get_single_num();
	joint[1] = get_single_num();
	joint[2] = get_single_num();
	index = -1;
	accessible = true;
}

void Read_data::receieve() 
{
	if(index < size_of_data ) 
	{	
		HAL_UART_Receive_IT(&huart5, &ch, 1);
		if(ch != '\n')
		{
			index++;
			temp[index] = ch;
		}
	}
	if(index == (size_of_data-1) )
	{
			assign_num();
	}
}

