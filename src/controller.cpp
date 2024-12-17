#include <Arduino.h>
#include "controller.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"
#define ESP32DEBUGGING
#include <ESP32Logger.h>

std::mutex controller_mtx;
std::atomic<bool> controller_running(true);

#define BAT_CALIB_REAL_VOLTAGE  3691      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           1377      // adc-value measured by mainboard (value nr 5 on UART debug output)
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s


#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // in Volt: 
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow:       no beep
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow blink: no beep 
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red:          gently beep at this voltage level. [V*100/cell]. In this case 3.60 V/cell
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red blink:    fast beep. Your battery is almost empty. Charge now! [V*100/cell]. In this case 3.50 V/cell
#define BAT_DEAD                (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // All leds off: undervoltage poweroff. (while not driving) [V*100/cell]. In this case 3.37 V/cell


void Controller::periodicTask() {
    while (controller_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        taskYIELD();
        rtc_wdt_feed();
        esp_task_wdt_reset();        
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
    int maxAccReverse;
    int maxSpeedReverse;
    int maxSteering;
};

ControllerSettings globalSettings;

ControllerSettings settingsArray[] = {
    {20, 4, 10, 20, 4, 1023},  // Kid
    {100, 6, 20, 70, 6, 1023},  // Average
    {500, 10, 30, 70, 6, 1023}   // Advanced
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
    if (this->throttle < 15)
        return 0;
    return this->throttle;
}

void 
Controller::setDriveMode(DriveMode mode) {
    DBGCHK(Verbose, SERIAL_DEBUG_CONTROLLER, "Setting drive mode to %d\n", mode);
    switch (mode) {
        case MOTOR_OFF:
            DBGCHK(Info, SERIAL_DEBUG_CONTROLLER, "Motor off");
            break;
        case NEUTRAL:
            DBGCHK(Info, SERIAL_DEBUG_CONTROLLER, "Neutral");
            break;
        case DRIVE:
            if (!motorConnected) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Motor error, cannot switch to drive!");
                return;
                }
            if (abs(this->realRPM) > 50) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Cannot switch to drive while moving!");
                return;
            }
            DBGCHK(Info, SERIAL_DEBUG_CONTROLLER, "Drive");
            break;
        case REVERSE:
            if (!motorConnected) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Motor error, cannot switch to reverse!");
                return;
            }
            if (abs(this->realRPM) > 50) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Cannot switch to reverse while moving!");
                return;
            }
            DBGCHK(Info, SERIAL_DEBUG_CONTROLLER, "Reverse");
            break;
        case PARKING:
            if (!getMotorConnected()) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Motor error, cannot switch to parking!");
                return;
            }
            if (abs(this->realRPM) > 50) {
                DBGCHK(Error, SERIAL_DEBUG_CONTROLLER, "Cannot switch to parking while moving!");
                return;
            }
            DBGCHK(Info, SERIAL_DEBUG_CONTROLLER, "Parking");
            break;
        default:
            DBGCHK(Warning, SERIAL_DEBUG_CONTROLLER, "Invalid drive mode");
            return;
    }
    DBGCHK(Verbose, SERIAL_DEBUG_CONTROLLER, "really setting drive mode to %d", mode);
    this->driveMode = mode;
}

Controller::Controller(int mode)  : controller_running(true) {
    this->throttle = 0;
    this->brake = 0;
    this->steering = 0;
    this->leftSpeed = 0;
    this->rightSpeed = 0;
    this->driveMode = MOTOR_OFF;

    this->lightStates.brakeLight = false;
    this->lightStates.lowBeam = false;
    this->lightStates.highBeam = false;
    this->lightStates.indicatorLeft = false;
    this->lightStates.indicatorRight = false;
    this->lightStates.horn = false;
    this->espWorking = false;
    this->launchControlActive = false;
    this->indicatorOn = false;
    this->calculatedTorqueLeft = 0;
    this->calculatedTorqueRight = 0;
    this->realRPM = 0;
    this->simulatedRPM = 0;
    this->motorConnected = false;
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

void Controller::setMotorConnected(bool connected) {
    if (!connected) {
        if (this->driveMode == DRIVE || this->driveMode == REVERSE) {
            this->driveMode = NEUTRAL;
            return;
        }
    }
    this->motorConnected = connected;
}

void        
Controller::setMotorButton(bool motorButton) {
    if (motorButton) {
        if (this->driveMode == MOTOR_OFF) {
            this->driveMode = NEUTRAL;
//            this->driveMode = NEUTRAL;
        } else {
            this->driveMode = MOTOR_OFF;
        }
    }
}

void Controller::compute() {
    if (this->driveMode == MOTOR_OFF) {
        this->calculatedTorqueLeft = 0;
        this->calculatedTorqueRight = 0;
    }

    if (this->driveMode == NEUTRAL) {
        this->setSimulatedRPM(this->throttle);
    }
//    float wheelDiameterincm = 16.51;     // 16,5" Reifen
    float wheelDiameterincm = 20.32;     // 8" Reifen
    float wheelCircumference = wheelDiameterincm * 3.14159265359;

    if (this->driveMode == DRIVE) {
        int16_t rpm = this->getRealRPM();
        float speed = (rpm * wheelCircumference) / 6000.0;      //  Geschwindigkeit in km/h

        if (speed > globalSettings.maxSpeed) {  // Zwangsbremse!!!!
            this->calculatedTorqueLeft = -0;
            this->calculatedTorqueRight = -0;
        } else {
            int16_t throttle;
            static int16_t lastThrottle = 0;

            int16_t targetTorque = 0;
            int16_t leftTargetTorque = 0;
            static int16_t lastLeftTargetTorque = 0;
/*          if (this->brake > 0) {      // Bremsen
                if (abs(speed) < 0.5) {  // Standstill

                } else {    // Abbremsen

                }
                
            } else */
            throttle = this->getCompensatedThrottle();
            if (throttle > 10) {    // Beschleunigen
                targetTorque = throttle;
                leftTargetTorque = map(targetTorque,0,1023,0,globalSettings.maxAcceleration);
                if (leftTargetTorque < 5) {
                    leftTargetTorque = 0;
                }
                if ((throttle != lastThrottle) || (leftTargetTorque != lastLeftTargetTorque)) {
                    lastThrottle = throttle;
                    lastLeftTargetTorque = leftTargetTorque;
                    DBGCHK(Verbose, SERIAL_DEBUG_CONTROLLER_CALC, "Throttle: %d, target: %d", targetTorque, leftTargetTorque);
                }

            } else {
                if (throttle != lastThrottle) {
                    lastThrottle = throttle;
                    DBGCHK(Verbose, SERIAL_DEBUG_CONTROLLER_CALC, "Throttle: %d too low, ignoring", throttle);
                }
            }
            this->calculatedTorqueLeft = leftTargetTorque;
            this->calculatedTorqueRight = -leftTargetTorque;
        }

    } else if (this->driveMode == REVERSE) {
        int16_t rpm = this->getRealRPM();
        float speed = (abs(rpm) * wheelCircumference) / 6000.0;      //  Geschwindigkeit in km/h

        if (speed > globalSettings.maxSpeed) {  // Zwangsbremse!!!!
            this->calculatedTorqueLeft = -0;
            this->calculatedTorqueRight = -0;
        } else {
            int16_t targetTorque = 0;
            int16_t leftTargetTorque = 0;
           
        }
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
    if (this->getMotorConnected()) {
        return this->realRPM;
    }
    return 0;
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

