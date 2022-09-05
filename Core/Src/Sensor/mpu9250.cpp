#include "Sensor/mpu9250.h"
#include "Sensor/mpu9250_register_map.h"

Mpu9250::Mpu9250(/* args */)
{

}

Mpu9250::~Mpu9250()
{
}

void Mpu9250::set(SPI_HandleTypeDef* spi_h, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
    Spi_Sensor::set(spi_h, cs_port, cs_pin);
    if (read_register(WHO_AM_I_MPU9250) != 0x71)
        return;
    write_register(PWR_MGMT_1, 01);
	write_register(USER_CTRL, 0x20);
	write_register(PWR_MGMT_2, 0x00);
	write_register(ACCEL_CONFIG, 0x00);
	write_register(GYRO_CONFIG, 0x18);
	write_register(ACCEL_CONFIG2, 0x01);
	write_register(CONFIG, 0x01);
	write_register(SMPLRT_DIV, 0x00);
}

int16_t Mpu9250::read_value(uint8_t type)
{
	int16_t h = read_register(type);
	int16_t l = read_register(type);
	return ((h<<8 | l));
}

void Mpu9250::filter(float* a, float* g, Quaternion &q)
{
	
}

void Mpu9250::update(Dynamics &s)
{
	// float a[3] = {0};
	// float g[3] = {0};

	//read acceleration and gyro
	for (int i = 0; i < 3; i++)
		acce[i] = (float)(read_value(ACCEL_XOUT_H + i * 2) * 599) / (float)1000000;
	for (int i = 0; i < 3; i++)
		gyro[i] = (float)(read_value(GYRO_XOUT_H + i * 2) * 133) / (float)1000000;
}