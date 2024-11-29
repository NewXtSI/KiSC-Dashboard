#include <Arduino.h>
#include <WiFi.h>
#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include "gui.h"
#include <motor.h>
#include <controller.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"
#include "hardware.h"

Motor       motorc;
Controller  controller;

bool        motorButton = false;
bool        bStarted = false;
void sendSoundAndLightControl() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::SoundLightControl;
    message.soundAndLightControl.motorOn = controller.getDriveMode() == DriveMode::DRIVE || controller.getDriveMode() == DriveMode::REVERSE || controller.getDriveMode() == DriveMode::NEUTRAL;
    message.soundAndLightControl.throttle = controller.getCompensatedThrottle();
    
    if (controller.getThrottle() > 20) {
        message.soundAndLightControl.lights.highBeam = false;
        message.soundAndLightControl.lights.lowBeam = false;
    } else {
        message.soundAndLightControl.lights.highBeam = false;
        message.soundAndLightControl.lights.lowBeam = false;
    }
    message.soundAndLightControl.lights.indicatorLeft = controller.getIndicatorOn();
    message.soundAndLightControl.lights.indicatorRight = controller.getIndicatorOn();
    message.soundAndLightControl.volumeMain = 0.8;

    sendKiSCMessage(SOUND_LIGHT_CONTROLLER_MAC, message);
}

void sendMotorControl() {
    kisc::protocol::espnow::KiSCMessage message;
    message.motorControl.left.enable = false;
    message.motorControl.right.enable = false;
    
    message.motorControl.left.pwm = 0;
    message.motorControl.right.pwm = 0;

    message.motorControl.left.iMotMax = 10;
    message.motorControl.right.iMotMax = 10;


    message.command = kisc::protocol::espnow::Command::MotorControl;

    sendKiSCMessage(MOTOR_CONTROLLER_MAC, message);
}

void recCallback(kisc::protocol::espnow::KiSCMessage message) {
//    Serial.println("Received message");

    switch (message.command) {
    case kisc::protocol::espnow::Command::Ping:
        Serial.println("Ping message");
        break;
    case kisc::protocol::espnow::Command::MotorControl:
        Serial.println("Motor control message");
        break;
    case kisc::protocol::espnow::Command::MotorFeedback:
//        Serial.println("Motor feedback message");
        motorc.setVoltage(message.motorFeedback.batVoltage /100.0);
        motorc.setTemperature(message.motorFeedback.boardTemp / 10.0);
        Serial.printf("Voltage: %.2fV Temperature: %.2fC\n", motorc.getVoltage(), motorc.getTemperature());
        break;
    case kisc::protocol::espnow::Command::PeriphalControl:
        Serial.println("Periphal control message");
        break;
        // Pedale, Lenkung Buttons
    case kisc::protocol::espnow::Command::PeriphalFeedback:
        Serial.println("Periphal feedback message");
//        Serial.printf("Throttle: %d Motor Button: %d\n", message.peripheralFeedback.throttle, motorButton);
        controller.setThrottle(message.peripheralFeedback.throttle);
        controller.setMotorButton(message.peripheralFeedback.motorButton);
        break;
    case kisc::protocol::espnow::Command::SoundLightControl:
        Serial.println("Sound light control message");
        break;
    case kisc::protocol::espnow::Command::SoundLightFeedback:
        Serial.println("Sound light feedback message");
        break;
    case kisc::protocol::espnow::Command::BTControl:
        Serial.println("BT control message");
        break;
    case kisc::protocol::espnow::Command::BTFeedback:
        Serial.println("BT feedback message");
        break;
    default:
        Serial.printf("Unknown message %d\n", message.command); 
        break;
    }

}

void guiTask(void* pvParameters) {
    Serial.println("Starting GUI task");
    GUI gui(motorc, controller);
    Serial.println("GUI initialized");
    gui.init();
    Serial.println("GUI created");
    gui.create();
    Serial.println("GUI task loop started");
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
    Serial.println("Starting Comm task");
    onKiSCMessageReceived(recCallback);
    initESPNow();
    Serial.printf("\n\n---- Dash Controller ----\n");
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
    while (true) {
        loopESPNow();
        vTaskDelay(pdMS_TO_TICKS(5)); // Adjust the delay as needed
        rtc_wdt_feed();
        esp_task_wdt_reset();
        if (iSendInterval++ > 100) {
            iSendInterval = 0;
//            sendSoundAndLightControl();
//            sendMotorControl();
        }
        if (millis() - lastHeartbeat > 10000) {
//            Serial.printf("Sending heartbeat\n");
            sendHeartbeat();
            lastHeartbeat = millis();
        }
    }
}

void setup() {
  // put your setup code here, to run once:
    esp_task_wdt_init(60, true);
    esp_task_wdt_deinit();
    delay(500);
    Serial.begin(115200);
    //Serial.setDebugOutput(true);
    delay(500);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);
    xTaskCreatePinnedToCore(guiTask, "GUI Task", 4096*3, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(commTask, "COMM Task", 4096, NULL, 3, NULL, 1);

}

void loop() {
    delay(1);
}