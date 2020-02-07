#ifndef x_mpu9250_h
#define x_mpu9250_h

#include <MPU9250.h>      // IMU / MPU-9250 lib (see https://github.com/bolderflight/MPU9250)

// small extension of IMU / MPU-9250 lib, adds method for sleep mode
class xMPU9250 : public MPU9250 {

    public:

        xMPU9250(TwoWire &bus,uint8_t address) :  MPU9250(bus, address) {}

        void setSleepEnabled() {
            writeRegister(PWR_MGMNT_1, PWR_CYCLE);
        }
};

#endif