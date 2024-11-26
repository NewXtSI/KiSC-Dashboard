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
    float getCurrent(uint8_t motor);
    double getSpeed(uint8_t motor);

    bool isConnected() { return connected; }
private:
    std::mutex motor_mtx;
    int leftSpeed;
    int rightSpeed;
    int steering;
    bool connected;

    std::thread periodicThread;
    std::atomic<bool> motor_running;

    float voltage;
    float current[2];
    double speed[2];

    void periodicTask();
    void updateStatistics();
};

#endif
