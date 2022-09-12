#include "Sensor/bar02.h"

// sensor address
const uint8_t MS5837_ADDR = 0x76;
// sensor command byte
uint8_t MS5837_RESET = 0x1E;
uint8_t MS5837_ADC_READ = 0x00;
uint8_t MS5837_PROM_READ = 0xA0;
uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
uint8_t MS5837_CONVERT_D2_8192 = 0x5A;
const float fluid_density = 997;

void Bar02::reset(){
    HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &(MS5837_RESET), 1, 10);
}

bool Bar02::set(I2C_HandleTypeDef* handler){
    h = handler;
    reset();
    HAL_Delay(10);

    // Read calibration values amd CRC
    for (uint8_t i=0;i<8;i++){
        uint8_t buffer[2]={};
        uint8_t cmd = MS5837_PROM_READ+i*2;
        HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &cmd, 1, 1000);
        HAL_I2C_Master_Receive(h, (MS5837_ADDR <<1), buffer, 2, 1000);
        C[i] = (buffer[0]<<8) | (buffer[1]);
    }

    // Verify CRC
    uint8_t crcRead = C[0]>>12;
    uint8_t crcCalculated = crc4(C);
    if (crcRead != crcCalculated)
        return false;

    return true;
}

float Bar02::read_value(){
    uint8_t buffer[3]={};

    // D1 conversion
    HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &MS5837_CONVERT_D1_8192, 1, 1000);
    HAL_Delay(20);
    HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &MS5837_ADC_READ, 1, 1000);
    HAL_I2C_Master_Receive(h, ((MS5837_ADDR <<1)), buffer, 3, 1000);
    D1_pres = (buffer[0]<<16) | (buffer[1]<<8) | buffer[2];

    buffer[0]=0; buffer[1]=0; buffer[2]=0;
    // D2 conversion
    HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &MS5837_CONVERT_D2_8192, 1, 1000);
    HAL_Delay(20);
    HAL_I2C_Master_Transmit(h, (MS5837_ADDR <<1), &MS5837_ADC_READ, 1, 1000);
    HAL_I2C_Master_Receive(h, ((MS5837_ADDR <<1)), buffer, 3, 1000);
    D2_temp = (buffer[0]<<16) | (buffer[1]<<8) | buffer[2]; 

    calculate();

    // return depth (from pressure)
    return (P -101300)/(fluid_density*9.80665);
}


void Bar02::calculate(){
    int32_t dT = 0;   int64_t SENS = 0;
    int64_t OFF = 0;  int32_t SENSi = 0;
	int32_t OFFi = 0; int32_t Ti = 0; 
    int64_t OFF2 = 0; int64_t SENS2 = 0;

    dT = D2_temp-uint32_t(C[5])*256l;
	SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
	OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
	P = (D1_pres*SENS/(2097152l)-OFF)/(32768l);

    // Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

    // Second order conversion
    if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
	}

    //Calculate pressure and temp second order
    OFF2 = OFF-OFFi;           
	SENS2 = SENS-SENSi;
    TEMP = (TEMP-Ti);
    P = (((D1_pres*SENS2)/2097152l-OFF2)/32768l);
}


uint8_t Bar02::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}


/*

// in main : (USER CODE BEGIN 1)
    Bar02 depth_sensor;

// in USER CODE BEGIN 2
    bool tmp = depth_sensor.set(&hi2c1);

// in while loop  (USER CODE END WHILE)
    uart_buf_len = sprintf(uart_buf, "Depth: %f \r\n", depth_sensor.read_value());
    HAL_UART_Transmit(&huart4, (uint8_t*) uart_buf, uart_buf_len, 1000);
*/