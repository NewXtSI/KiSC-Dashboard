#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "globals.h"
#include <mutex>
#include <atomic>
#include <thread>

typedef enum {
    MOTOR_OFF = 0,
    PARKING,
    NEUTRAL,
    DRIVE,
    REVERSE
} DriveMode;

typedef struct {
    bool    lowBeam;
    bool    highBeam;
    bool    brakeLight;
    bool    indicatorLeft;
    bool    indicatorRight;
    bool    horn;

} LightStates;
// The controller should have inputs for throttle and brake (values range from 0 to 1023) and steering (-1023 to 1023, left to right)
class Controller {
public:
    Controller(int mode = 1);  // Default to kid mode
    ~Controller();
    void setSettings(int mode);
    void compute();

    void                setThrottle(int throttle);
    int                 getThrottle();
    int                 getCompensatedThrottle();

    void                setBrake(int brake);
    int                 getBrake();
    
    void                setSteering(int steering);
    int                 getSteering();

    // return -1023 to 1023, MOTOR_FLYWHEEL for flywheel
    int                 getLeftSpeed();
    int                 getRightSpeed();

    void                setDriveMode(DriveMode mode);
    DriveMode           getDriveMode();

    void                setSimulatedRPM(int rpm);
    int                 getSimulatedRPM();

    void                setRealRPM(int rpm);
    int                 getRealRPM();



    int                 getRPM();
    void                setMotorButton(bool motorButton);
    bool                getIndicatorOn() { return indicatorOn; }
    LightStates         getLightStates() { return lightStates; }
    bool                getESPWorking() { return espWorking; }
    bool                getLaunchControlActive() { return launchControlActive; }

    int                 getCalculatedTorqueLeft() { return calculatedTorqueLeft; }
    int                 getCalculatedTorqueRight() { return calculatedTorqueRight; }
    void                setMotorConnected(bool connected);
    bool                getMotorConnected() { return motorConnected; }

    void                setYPR(float *ypr) { memcpy(this->ypr, ypr, sizeof(float) * 3); }
    void                setAcc(float *acc) { memcpy(this->acc, acc, sizeof(float) * 3); }
private:
    LightStates         lightStates;
    std::thread         periodicThread;
    std::atomic<bool>   controller_running;
    std::mutex          controller_mtx;

    DriveMode           driveMode;
    int                 throttle;
    int                 brake;
    int                 steering;
    int                 leftSpeed;
    int                 rightSpeed;
    bool                motorConnected;
    int                 realRPM;
    int                 simulatedRPM;


    int                 calculatedTorqueLeft;
    int                 calculatedTorqueRight;
    bool                indicatorOn;
    bool                espWorking;
    bool                launchControlActive;


    float               ypr[3];
    float               acc[3];
    void                periodicTask();
};

#endif
