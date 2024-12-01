#pragma once
#include "screen.h"
#include "controller.h"

class Dash : public Screen {
public:
    Dash();
    void create() override;
    void setSpeed(float speed) { this->speed = speed; }
    void setThrottle(int throttle) { this->throttle = throttle; }
    void setPower(float power) { this->power = power; }
    void setTemperature(float temperature) { this->temperature = temperature; }
    void setBattery(float battery) { this->battery = battery; }
    void setDriveMode(DriveMode mode) { this->driveMode = mode; }
private:
    float       speed;
    int         throttle;
    float       power;
    float       temperature;
    float       battery;
    bool        motorConnected;
    DriveMode  driveMode;
    lv_obj_t * battery_bar;
    lv_obj_t * temperature_bar;
    lv_obj_t * throttle_bar;
    lv_obj_t * power_bar;
    lv_obj_t * speed_label;
    lv_obj_t * drivemode_label;
    
    lv_obj_t * indicator_left_img;
    lv_obj_t * indicator_right_img;

    lv_obj_t * check_engine_img;
    lv_obj_t * esp_img;
    lv_obj_t * launch_img;
    lv_obj_t * park_img;
    lv_obj_t * beam_img;
    lv_obj_t * fuel_power_img;
    lv_obj_t * fuel_power_16_img;

    void refreshStates();
    void init() override;
    void update() override;
};
