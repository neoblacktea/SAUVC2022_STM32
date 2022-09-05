#ifndef MPU9250_H
#define MPU9250_H

#include "spi_sensor.h"
#include "Datatype/dynamics.h"

class Mpu9250:Spi_Sensor
{
private:
	Quaternion q_est = {1, 0, 0, 0};

public:
    Mpu9250(/* args */);
    ~Mpu9250();
    void set(SPI_HandleTypeDef* spi_h, GPIO_TypeDef* cs_port, uint16_t cs_pin);
    void set_memory_dress(uint8_t num, uint8_t address);
	int16_t read_value(uint8_t type);
	void filter(float* a, float* g, Quaternion &q);
	void update(Dynamics &s);

	float acce[3] = {0};
	float gyro[3] = {0};
};

#endif