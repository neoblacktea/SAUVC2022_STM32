#include "stm32f4xx.h"
// #include "i2c.h"

class Bar02{
    public:
        void reset();
        bool set(I2C_HandleTypeDef*);
        float read_value();
        void calculate();
        uint8_t crc4(uint16_t*);

    private:
        uint16_t C[8];
        uint32_t D1_pres, D2_temp;
        int32_t TEMP;
        int32_t P;
        I2C_HandleTypeDef* h;
};
