#include <Arduino.h>
#include "controller.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

std::mutex controller_mtx;
std::atomic<bool> controller_running(true);

void Controller::periodicTask() {
    while (controller_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> lock(controller_mtx);
        compute();

        // Indicator On toggled alle 1500 ms
        static uint32_t lastIndicatorToggle = 0;
        if (millis() - lastIndicatorToggle > 750) {
            lastIndicatorToggle = millis();
            indicatorOn = !indicatorOn;
        }
    }
}

struct ControllerSettings {
    int maxAcceleration;
    int maxSpeed;
    int maxBrake;
    int maxSteering;
};

ControllerSettings globalSettings;

ControllerSettings settingsArray[] = {
    {10, 300, 10, 1023},  // Kid
    {20, 600, 20, 1023},  // Average
    {30, 900, 30, 1023}   // Advanced
};

// Initialize the global settings to kid safe values
void initControllerSettings() {
    globalSettings = settingsArray[0];
}

int         
Controller::getCompensatedThrottle() {
    if (this->driveMode == MOTOR_OFF) {
        return 0;
    }
    return this->throttle;
}

void 
Controller::setDriveMode(DriveMode mode) {
    this->driveMode = mode;
}

Controller::Controller(int mode)  : controller_running(true) {
    this->throttle = 0;
    this->brake = 0;
    this->steering = 0;
    this->leftSpeed = 0;
    this->rightSpeed = 0;
    this->flywheel = false;
    this->driveMode = MOTOR_OFF;
    setSettings(mode);

    periodicThread = std::thread(&Controller::periodicTask, this);
}

void Controller::setSettings(int mode) {
    if (mode >= 0 && mode < 3) {
        globalSettings = settingsArray[mode];
    } else {
        // Default to kid mode if invalid mode is provided
        globalSettings = settingsArray[0];
    }
}

void Controller::demonstrate() {
    // Run a fake loop via a timer to demonstrate the controller

}

void        
Controller::setMotorButton(bool motorButton) {
    if (motorButton) {
        if (this->driveMode == MOTOR_OFF) {
            this->driveMode = NEUTRAL;
        } else {
            this->driveMode = MOTOR_OFF;
        }
    }
}

// In the periodic task the controller should compute the speeds based on the throttle input.
// If the throttle input is 0, the controller should enter flywheel mode, if the actual speed is above a certain threshold.
// The controller should also take the brake input into account. If the brake input is above a certain threshold, the controller should brake.
// The speed jumps to high at the motors, so a maximum acceleration should be taken into account.
// The acceleration should not be linear, but should be based on the difference between the current speed and the desired speed.
// The brake should not be fixed, should be a slowdown of the speed with a given factor
void Controller::compute() {
    int desiredSpeed = 0;
    int acceleration = 0;
    int brake = 0;

    if (this->throttle == 0) {
        if (this->leftSpeed > 0) {
            this->leftSpeed -= 10;
        } else {
            this->leftSpeed = 0;
        }
        if (this->rightSpeed > 0) {
            this->rightSpeed -= 10;
        } else {
            this->rightSpeed = 0;
        }
    } else {
        desiredSpeed = this->throttle * globalSettings.maxSpeed / 1023;
        acceleration = (desiredSpeed - this->leftSpeed) * globalSettings.maxAcceleration / 1023;
        if (acceleration > globalSettings.maxAcceleration) {
            acceleration = globalSettings.maxAcceleration;
        }
        if (acceleration < -globalSettings.maxAcceleration) {
            acceleration = -globalSettings.maxAcceleration;
        }
        this->leftSpeed += acceleration;
        this->rightSpeed += acceleration;
    }

    if (this->brake > 0) {
        brake = this->brake * globalSettings.maxBrake / 1023;
        if (this->leftSpeed > brake) {
            this->leftSpeed -= brake;
        } else {
            this->leftSpeed = 0;
        }
        if (this->rightSpeed > brake) {
            this->rightSpeed -= brake;
        } else {
            this->rightSpeed = 0;
        }
    }

    if (this->leftSpeed > globalSettings.maxSpeed) {
        this->leftSpeed = globalSettings.maxSpeed;
    }
    if (this->rightSpeed > globalSettings.maxSpeed) {
        this->rightSpeed = globalSettings.maxSpeed;
    }
    if (this->leftSpeed < 0) {
        this->leftSpeed = 0;
    }
    if (this->rightSpeed < 0) {
        this->rightSpeed = 0;
    }
}

Controller::~Controller() {
    // nothing to do
}

void Controller::setThrottle(int throttle) {
    this->throttle = throttle;
}

int Controller::getThrottle() {
    return this->throttle;
}

void Controller::setBrake(int brake) {
    this->brake = brake;
}

int Controller::getBrake() {
    return this->brake;
}

void Controller::setSteering(int steering) {
    this->steering = steering;
}

int Controller::getSteering() {
    return this->steering;
}

int Controller::getLeftSpeed() {
    return this->leftSpeed;
}

int Controller::getRightSpeed() {
    return this->rightSpeed;
}

void Controller::setLeftSpeed(int speed) {
    this->leftSpeed = speed;
}

void Controller::setRightSpeed(int speed) {
    this->rightSpeed = speed;
}

void Controller::setFlywheel(bool flywheel) {
    this->flywheel = flywheel;
}

bool Controller::getFlywheel() {
    return this->flywheel;
}

void Controller::setSimulatedRPM(int rpm) {
    this->simulatedRPM = rpm;
}

int Controller::getSimulatedRPM() {
    return this->simulatedRPM;
}

void Controller::setRealRPM(int rpm) {
    this->realRPM = rpm;
}

int Controller::getRealRPM() {
    return this->realRPM;
}

int Controller::getRPM() {
    if (this->driveMode == MOTOR_OFF) {
        return 0;
    }
    if (this->driveMode == NEUTRAL) {
        return getSimulatedRPM();
    }
    return getRealRPM();
}

DriveMode Controller::getDriveMode() {
    return this->driveMode;
}

