#pragma once
#include <Arduino.h>

typedef struct {
    uint16_t  throttlePosition;
    uint16_t  brakePosition;
    uint16_t  steeringPosition;

    uint8_t   motorStartStopbutton:1;
    uint8_t   gear_up:1;
    uint8_t   gear_down:1;

    uint8_t   horn:1;
} PeripheralData;

typedef struct {
    uint8_t   motorState;               // 0 = off, 1 = on
    uint16_t  motorSpeed;               // Simulated RPM
    uint8_t   Volume;
    
    uint8_t   SoundsEnabled:1;
    uint8_t   A2DPEnabled:1;
    
    uint8_t   IndicatorLeft:1;
    uint8_t   IndicatorRight:1;
    uint8_t   RearLight:1;
    uint8_t   FrontLowBeam:1;
    uint8_t   FrontHighBeam:1;
    uint8_t   BrakeLight:1;
    uint8_t   ReverseLight:1;

    uint8_t   Horn:1;

} SoundAndLightData;

typedef enum {
    I2CPeripheralController = 0x10,
    I2CSoundAndLightController = 0x20,
} I2CDevices;

class I2CMaster {
public:
    I2CMaster();
    void init();
    bool isDeviceConnected(I2CDevices device);
    void refreshDevices();
    void refreshDevice(I2CDevices device);
private:    
    void write(uint8_t address, uint8_t reg, uint8_t data);
    void read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t len);
};  // I2CMaster