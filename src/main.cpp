#include <Arduino.h>
#include <WiFi.h>
//#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include "../KiSC-ESP-Now-Protocol/include/kiSCprotov2.h"

#include "gui.h"
#include <motor.h>
#include <controller.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"
#include "hardware.h"
#include <OneButton.h>

#include "globals.h"
#define ESP32DEBUGGING
#include <ESP32Logger.h>


#if 0

Motor       motorc;
Controller  controller;

bool        motorButton = false;
bool        bStarted = false;

// Declare and initialize
OneButton shiftUpButton = OneButton(
  35,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);
OneButton shiftDownButton = OneButton(
  0,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

void handleShiftDownClick();
void handleShiftUpClick();

void sendSoundAndLightControl() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::SoundLightControl;
    message.soundAndLightControl.motorOn = controller.getDriveMode() == DriveMode::DRIVE || controller.getDriveMode() == DriveMode::REVERSE || controller.getDriveMode() == DriveMode::NEUTRAL;
    message.soundAndLightControl.throttle = controller.getCompensatedThrottle();
    message.soundAndLightControl.rpm = map(controller.getRPM(), 0, 1000, 0, 500);

    if (controller.getDriveMode() == DriveMode::REVERSE) {
        message.soundAndLightControl.lights.reverseLight = true;
    } else {
        message.soundAndLightControl.lights.reverseLight = false;
    }
    if (controller.getThrottle() > 20) {
        message.soundAndLightControl.lights.highBeam = false;
        message.soundAndLightControl.lights.lowBeam = false;
    } else {
        message.soundAndLightControl.lights.highBeam = false;
        message.soundAndLightControl.lights.lowBeam = false;
    }
    message.soundAndLightControl.lights.indicatorLeft = controller.getIndicatorOn();
    message.soundAndLightControl.lights.indicatorRight = controller.getIndicatorOn();
    message.soundAndLightControl.volumeMain = 0.3;

    sendKiSCMessage(SOUND_LIGHT_CONTROLLER_MAC, message);
}

bool remoteControlActive = false;

void sendPeriphalControl() {
    if (remoteControlActive) {
        kisc::protocol::espnow::KiSCMessage message;
        message.command = kisc::protocol::espnow::Command::PeriphalControl;
        message.peripheralControl.steering = controller.getSteering();
        sendKiSCMessage(PERIPHERAL_CONTROLLER_MAC, message);
    }
}

void sendMotorControl() {
    kisc::protocol::espnow::KiSCMessage message;
    message.motorControl.left.enable = true;
    message.motorControl.right.enable = true;
    
    message.motorControl.left.pwm = motorc.getDesiredTorqueLeft();
    message.motorControl.right.pwm = motorc.getDesiredTorqueRight();

    message.motorControl.left.iMotMax = 10;
    message.motorControl.left.nMotMax = 200;
    message.motorControl.left.cruiseCtrlEna = false;
    message.motorControl.left.cruiseMotTgt = 0;
    message.motorControl.left.electricBrakeFactor = 127;
    message.motorControl.left.mode = 2;
    message.motorControl.left.type = 2;

    message.motorControl.right.iMotMax = 10;
    message.motorControl.right.nMotMax = 200;
    message.motorControl.right.cruiseCtrlEna = false;
    message.motorControl.right.cruiseMotTgt = 0;
    message.motorControl.right.electricBrakeFactor = 127;
    message.motorControl.right.mode = 2;
    message.motorControl.right.type = 2;
    
    message.command = kisc::protocol::espnow::Command::MotorControl;

    sendKiSCMessage(MOTOR_CONTROLLER_MAC, message);
}

void recCallback(kisc::protocol::espnow::KiSCMessage message) {
//    Serial.println("Received message");

    switch (message.command) {
    case kisc::protocol::espnow::Command::Ping:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Ping message");
        break;
    case kisc::protocol::espnow::Command::MotorControl:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Motor control message");
        break;
    case kisc::protocol::espnow::Command::MotorFeedback:
//        Serial.println("Motor feedback message");
        motorc.setVoltage(message.motorFeedback.batVoltage / 100.0);
        motorc.setTemperature(message.motorFeedback.boardTemp / 10.0);
        motorc.setCharging(message.motorFeedback.bCharging);
        motorc.setConnected(message.motorFeedback.motorboardConnected);
        motorc.setRpm(message.motorFeedback.left.speed, message.motorFeedback.right.speed);
        controller.setRealRPM(message.motorFeedback.left.speed);
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Voltage: %.2fV Temperature: %.2fC", motorc.getVoltage(), motorc.getTemperature());
        break;
    case kisc::protocol::espnow::Command::PeriphalControl:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Periphal control message");
        break;
        // Pedale, Lenkung Buttons
    case kisc::protocol::espnow::Command::PeriphalFeedback:
        float acc[3];
        float ypr[3];
//        Serial.println("Periphal feedback message");
//        Serial.printf("Throttle: %d Motor Button: %d\n", message.peripheralFeedback.throttle, motorButton);
        if (!remoteControlActive) {
            controller.setThrottle(message.peripheralFeedback.throttle);
            controller.setBrake(message.peripheralFeedback.brake);
            controller.setMotorButton(message.peripheralFeedback.motorButton);
        }
        acc[0] = message.peripheralFeedback.acc[0] / 100.0;
        acc[1] = message.peripheralFeedback.acc[1] / 100.0;
        acc[2] = message.peripheralFeedback.acc[2] / 100.0;
        ypr[0] = message.peripheralFeedback.ypr[0] / 100.0;
        ypr[1] = message.peripheralFeedback.ypr[1] / 100.0;
        ypr[2] = message.peripheralFeedback.ypr[2] / 100.0;

        controller.setAcc(acc);
        controller.setYPR(ypr);
        static bool bLastActive = false;
        if (bLastActive != message.peripheralFeedback.nfcCardActive) {
            if (message.peripheralFeedback.nfcCardActive) {
                char uid[20];
                sprintf(uid, "%02X %02X %02X %02X", message.peripheralFeedback.nfcCardUID[0], message.peripheralFeedback.nfcCardUID[1], message.peripheralFeedback.nfcCardUID[2], message.peripheralFeedback.nfcCardUID[3]);
                DBGLOG(Verbose, "NFC Card active UID: %s", uid);                
            } else {
                DBGLOG(Verbose, "NFC Card not active");
            }
            bLastActive = message.peripheralFeedback.nfcCardActive;
        }
        break;
    case kisc::protocol::espnow::Command::RemoteFeedback:
        if (message.remoteFeedback.controllerConnected) {
            DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Remote feedback message, controller connected");
            remoteControlActive = true;
            controller.setThrottle(message.remoteFeedback.throttle);
            controller.setBrake(message.remoteFeedback.brake);
            controller.setMotorButton(message.remoteFeedback.motorButton);
            if (message.remoteFeedback.gearUpButton) {
                handleShiftUpClick();
            } else if (message.remoteFeedback.gearDownButton) {
                handleShiftDownClick();
            }
        } else {
            DBGCHK(Warning, SERIAL_DEBUG_KISCMESSAGES, "Remote feedback message, controller not connected");
            if (remoteControlActive) {  // Shutdown!!!!
                controller.setThrottle(0);
                controller.setBrake(0);
                remoteControlActive = false;
            }
        }
        break;
    case kisc::protocol::espnow::Command::SoundLightControl:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Sound light control message");
        break;
    case kisc::protocol::espnow::Command::SoundLightFeedback:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "Sound light feedback message");
        break;
    case kisc::protocol::espnow::Command::BTControl:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "BT control message");
        break;
    case kisc::protocol::espnow::Command::BTFeedback:
        DBGCHK(Verbose, SERIAL_DEBUG_KISCMESSAGES, "BT feedback message");
        break;
    default:
        DBGCHK(Warning, SERIAL_DEBUG_KISCMESSAGES, "Unknown message %d", message.command);
//        Serial.printf("Unknown message %d\n", message.command); 
        break;
    }

}

void guiTask(void* pvParameters) {
    DBGCHK(Info, SERIAL_DEBUG_GUI, "Starting GUI task");
    GUI gui(motorc, controller);
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "GUI initialized");
    gui.init();
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "GUI created");
    gui.create();
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "GUI task loop started");
    while (true) {

        gui.update();

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust the delay as needed
        rtc_wdt_feed();
        esp_task_wdt_reset();
    }
}

void sendHeartbeat() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::Ping;
    sendKiSCMessage(MOTOR_CONTROLLER_MAC, message);
//    sendKiSCMessage(SOUND_LIGHT_CONTROLLER_MAC, message);
//    sendKiSCMessage(PERIPHERAL_CONTROLLER_MAC, message);
}

uint32_t lastHeartbeat = 0;

void commTask(void* pvParameters) {
    int iSendInterval = 0;
    DBGLOG(Verbose, "Starting Comm task");
    onKiSCMessageReceived(recCallback);
    initESPNow();
    DBGLOG(Info, "---- Dash Controller ----");
    DBGLOG(Info, "MAC address: %s\n", WiFi.macAddress().c_str());
    while (true) {
        loopESPNow();
        vTaskDelay(pdMS_TO_TICKS(5)); // Adjust the delay as needed
        rtc_wdt_feed();
        esp_task_wdt_reset();
        if (iSendInterval++ > 20) {     // Target: Alle 100ms
            iSendInterval = 0;
            sendSoundAndLightControl();
            sendMotorControl();
            sendPeriphalControl();
        }
        if (millis() - lastHeartbeat > 10000) {
//            Serial.printf("Sending heartbeat\n");
            sendHeartbeat();
            lastHeartbeat = millis();
        }
    }
}

void handleShiftUpClick() {
    DBGLOG(Verbose, "Shiftup Button clicked");
    if ((controller.getDriveMode() != DriveMode::MOTOR_OFF) && (controller.getDriveMode() != DriveMode::PARKING)) {
        if (controller.getDriveMode() == DriveMode::NEUTRAL) {
            DBGLOG(Verbose, "Switching to drive");
            controller.setDriveMode(DriveMode::DRIVE);
        } else if (controller.getDriveMode() == DriveMode::REVERSE) {
            DBGLOG(Verbose, "Switching to neutral");
            controller.setDriveMode(DriveMode::NEUTRAL);
        } else {
            DBGLOG(Verbose, "Already in drive!");
        }
    } else if (controller.getDriveMode() == DriveMode::PARKING) {
        DBGLOG(Verbose, "Parking mode");
    } else {
        DBGLOG(Verbose, "Motor off");
    }
}

void handleShiftDownClick() {
    DBGLOG(Verbose, "Shiftdown Button clicked");
    if ((controller.getDriveMode() != DriveMode::MOTOR_OFF) && (controller.getDriveMode() != DriveMode::PARKING)) {
        if (controller.getDriveMode() == DriveMode::NEUTRAL) {
            DBGLOG(Verbose, "Switching to reverse");
            controller.setDriveMode(DriveMode::REVERSE);
        } else if (controller.getDriveMode() == DriveMode::DRIVE) {
            DBGLOG(Verbose, "Switching to neutral");
            controller.setDriveMode(DriveMode::NEUTRAL);
        } else {
            DBGLOG(Verbose, "Already in reverse!");
        }

    } else if (controller.getDriveMode() == DriveMode::PARKING) {
        DBGLOG(Verbose, "Parking mode");
    } else {
        DBGLOG(Verbose, "Motor off");
    }

}

#endif

KiSCProtoV2Master kiscProto("KiSCProtoV2Master");
void setup() {
  // put your setup code here, to run once:
    esp_task_wdt_init(60, true);
    esp_task_wdt_deinit();
    delay(500);
    Serial.begin(115200);
    DBGINI(&Serial)
    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
  //    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
    DBGLEV(SERIAL_DEFAULT_LOGLEVEL)
    if (SERIAL_OUTPUT_ENABLED) {
        DBGSTA
    }
    DBGLOG(Info, "---------------------------------------------------------------"
                "---------")
    DBGLOG(Info, "Enabled debug levels:")
    DBGLOG(Error, "Error")
    DBGLOG(Warning, "Warning")
    DBGLOG(Info, "Info")
    DBGLOG(Verbose, "Verbose")
    DBGLOG(Debug, "Debug")
    DBGLOG(Info, "---------------------------------------------------------------"
               "---------")    
kiscProto.start();
    //Serial.begin(115200);
   //Serial.setDebugOutput(true);
#if 0   
    delay(500);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);
    shiftUpButton.attachClick(handleShiftUpClick);
    shiftDownButton.attachClick(handleShiftDownClick);
    xTaskCreatePinnedToCore(guiTask, "GUI Task", 4096*3, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(commTask, "COMM Task", 4096, NULL, 3, NULL, 1);
#endif

}

void loop() {
    delay(1);
#if 0    
    controller.setMotorConnected(motorc.isConnected());
    motorc.setDesiredTorqueLeft(controller.getCalculatedTorqueLeft());
    motorc.setDesiredTorqueRight(controller.getCalculatedTorqueRight());
    shiftDownButton.tick();
    shiftUpButton.tick();
#endif    
}