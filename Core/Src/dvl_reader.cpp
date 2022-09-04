#include "dvl_reader.h"

Dvl_reader::Dvl_reader()
{
	velX = 0;
	velY = 0;
	velZ1 = 0;
	velZ2 = 0;
	delim = 165;
	for(int i = 0; i<222; i++)
		input[i] = 0;
}

Dvl_reader::~Dvl_reader()
{
	delete[] input;
}

float Dvl_reader::convert(int start_byte)
{
	float f;
	for(int i=0;i<4;i++)
		((char*)&f)[i] = input[start_byte+i];
	return f;	
}

void  Dvl_reader::update()
{
	velX = convert(start_of_velX);
	velY = convert(start_of_velY);
	velZ1 = convert(start_of_velZ1);
	velZ2 = convert(start_of_velZ2);
}	