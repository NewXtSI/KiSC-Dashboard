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
    Controller(int mode = 0);  // Default to kid mode
    ~Controller();
    void setSettings(int mode);
    void compute();

    // set the throttle
    void        setThrottle(int throttle);
    // get the throttle
    int         getThrottle();
    int         getCompensatedThrottle();
    // set the brake
    void        setBrake(int brake);
    // get the brake
    int         getBrake();
    // set the steering
    void        setSteering(int steering);
    // get the steering
    int         getSteering();

    // return -1023 to 1023, MOTOR_FLYWHEEL for flywheel
    int         getLeftSpeed();
    int         getRightSpeed();
    void        setLeftSpeed(int speed);
    void        setRightSpeed(int speed);

    void        setFlywheel(bool flywheel);
    bool        getFlywheel();

    void        demonstrate();
    void        setDriveMode(DriveMode mode);
    DriveMode   getDriveMode();

    void        setSimulatedRPM(int rpm);
    int         getSimulatedRPM();

    void        setRealRPM(int rpm);
    int         getRealRPM();

    int         getRPM();
    void        setMotorButton(bool motorButton);
    bool        getIndicatorOn() { return indicatorOn; }
    LightStates getLightStates() { return lightStates; }
    bool       getESPWorking() { return espWorking; }
    bool     getLaunchControlActive() { return launchControlActive; }
private:
    LightStates         lightStates;
    std::thread         periodicThread;
    std::atomic<bool>   controller_running;
    std::mutex          controller_mtx;

    DriveMode           driveMode;
    // throttle
    int                 throttle;
    // brake
    int                 brake;
    // steering
    int                 steering;
    int                 leftSpeed;
    int                 rightSpeed;
    bool                flywheel;
    int                 realRPM;
    int                 simulatedRPM;

    bool                indicatorOn;
    bool                espWorking;
    bool                launchControlActive;

    void                periodicTask();
};

#endif
