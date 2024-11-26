#include "motor.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

Motor::Motor() : motor_running(true), voltage(0.0f) {
    this->leftSpeed = 0;
    this->rightSpeed = 0;
    this->steering = 0;
    this->connected = false;

    periodicThread = std::thread(&Motor::periodicTask, this);
}

Motor::~Motor() {
    motor_running = false;
    if (periodicThread.joinable()) {
        periodicThread.join();
    }
}

void Motor::setSpeed(int speed) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    this->leftSpeed = speed;
    this->rightSpeed = speed;
}

void Motor::setLeftSpeed(int speed) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    this->leftSpeed = speed;
}

int Motor::getLeftSpeed() {
    std::lock_guard<std::mutex> lock(motor_mtx);
    return this->leftSpeed;
}

void Motor::setRightSpeed(int speed) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    this->rightSpeed = speed;
}

int Motor::getRightSpeed() {
    std::lock_guard<std::mutex> lock(motor_mtx);
    return this->rightSpeed;
}

void Motor::setSteering(int steering) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    this->steering = steering;
}

int Motor::getSteering() {
    std::lock_guard<std::mutex> lock(motor_mtx);
    return this->steering;
}

void Motor::sendSpeedAndSteering() {
}

void Motor::periodicTask() {
    while (motor_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> lock(motor_mtx);
        sendSpeedAndSteering();
        updateStatistics();
    }
}

void Motor::updateStatistics() {
}

float Motor::getVoltage() {
    std::lock_guard<std::mutex> lock(motor_mtx);
    if (this->isConnected()) {
        return this->voltage;
    }
    return 0.0f;
}

float Motor::getCurrent(uint8_t motor) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    if (motor < 2) {
        if (this->isConnected()) {
            return this->current[motor];
        }
    }
    return 0.0f;
}

float Motor::getTemperature() {
    std::lock_guard<std::mutex> lock(motor_mtx);
    if (this->isConnected()) {
        return this->temperature;
    }
    return 0.0f;
}

double Motor::getSpeed(uint8_t motor) {
    std::lock_guard<std::mutex> lock(motor_mtx);
    if (motor < 2) {
        if (this->isConnected()) {
        return this->speed[motor];
        }
    }
    return 0.0;
}
