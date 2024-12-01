#ifndef MOTOR_H
#define MOTOR_H

#include <thread>
#include <mutex>
#include <atomic>

class Motor {
public:
    Motor();
    ~Motor();
    void setSpeed(int speed);
    void setLeftSpeed(int speed);
    int getLeftSpeed();
    void setRightSpeed(int speed);
    int getRightSpeed();
    void setSteering(int steering);
    int getSteering();
    void sendSpeedAndSteering();
    float getVoltage();
    float getTemperature();
    float getCurrent(uint8_t motor);
    double getSpeed(uint8_t motor);
    void setRpm(int rpmLeft, int rpmRight);

    bool isConnected() { return this->connected; }
    void setConnected(bool connected) { this->connected = connected; }
    void setVoltage(float voltage) { this->voltage = voltage; }
    void setTemperature(float temperature) { this->temperature = temperature; }
    void setDesiredTorqueLeft(int torque) { this->desiredTorqueLeft = torque; }
    void setDesiredTorqueRight(int torque) { this->desiredTorqueRight = torque; }
    int getDesiredTorqueLeft() { return this->desiredTorqueLeft; }
    int getDesiredTorqueRight() { return this->desiredTorqueRight; }
private:
    std::mutex motor_mtx;
    int rpmLeft;
    int rpmRight;

    int desiredTorqueLeft;
    int desiredTorqueRight;
    int leftSpeed;
    int rightSpeed;
    int steering;
    bool connected;

    std::thread periodicThread;
    std::atomic<bool> motor_running;

    float voltage;
    float temperature;
    float current[2];
    double speed[2];

    void calcSpeed();
    void periodicTask();
    void updateStatistics();
};

#endif
