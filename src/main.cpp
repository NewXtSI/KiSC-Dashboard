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
#include <OneButton.h>

#define BLUETOOTH_HID           0

#if BLUETOOTH_HID
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_bt_main.h"
#include "esp_hidh_api.h"
#include "esp_hidh.h"
#endif

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
    message.motorControl.left.enable = true;
    message.motorControl.right.enable = true;
    
    message.motorControl.left.pwm = motorc.getDesiredTorqueLeft();
    message.motorControl.right.pwm = motorc.getDesiredTorqueRight();

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
        motorc.setVoltage(message.motorFeedback.batVoltage / 100.0);
        motorc.setTemperature(message.motorFeedback.boardTemp / 10.0);
        motorc.setConnected(message.motorFeedback.motorboardConnected);
        motorc.setRpm(message.motorFeedback.left.speed, message.motorFeedback.right.speed);
        controller.setRealRPM(message.motorFeedback.left.speed);
//        Serial.printf("Voltage: %.2fV Temperature: %.2fC\n", motorc.getVoltage(), motorc.getTemperature());
        break;
    case kisc::protocol::espnow::Command::PeriphalControl:
        Serial.println("Periphal control message");
        break;
        // Pedale, Lenkung Buttons
    case kisc::protocol::espnow::Command::PeriphalFeedback:
//        Serial.println("Periphal feedback message");
//        Serial.printf("Throttle: %d Motor Button: %d\n", message.peripheralFeedback.throttle, motorButton);
        controller.setThrottle(message.peripheralFeedback.throttle);
        controller.setBrake(message.peripheralFeedback.brake);
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
            sendSoundAndLightControl();
            sendMotorControl();
        }
        if (millis() - lastHeartbeat > 10000) {
//            Serial.printf("Sending heartbeat\n");
            sendHeartbeat();
            lastHeartbeat = millis();
        }
    }
}

void handleShiftUpClick() {
    Serial.println("Shiftup Button clicked");
    if ((controller.getDriveMode() != DriveMode::MOTOR_OFF) && (controller.getDriveMode() != DriveMode::PARKING)) {
        if (controller.getDriveMode() == DriveMode::NEUTRAL) {
            Serial.println("Switching to drive");
            controller.setDriveMode(DriveMode::DRIVE);
        } else if (controller.getDriveMode() == DriveMode::REVERSE) {
            Serial.println("Switching to neutral");
            controller.setDriveMode(DriveMode::NEUTRAL);
        } else {
            Serial.println("Already in drive!");
        }
    } else if (controller.getDriveMode() == DriveMode::PARKING) {
        Serial.println("Parking mode");
    } else {
        Serial.println("Motor off");
    }
}

void handleShiftDownClick() {
    Serial.println("Shiftdown Button clicked");
    if ((controller.getDriveMode() != DriveMode::MOTOR_OFF) && (controller.getDriveMode() != DriveMode::PARKING)) {
        if (controller.getDriveMode() == DriveMode::NEUTRAL) {
            Serial.println("Switching to reverse");
            controller.setDriveMode(DriveMode::REVERSE);
        } else if (controller.getDriveMode() == DriveMode::DRIVE) {
            Serial.println("Switching to neutral");
            controller.setDriveMode(DriveMode::NEUTRAL);
        } else {
            Serial.println("Already in reverse!");
        }

    } else if (controller.getDriveMode() == DriveMode::PARKING) {
        Serial.println("Parking mode");
    } else {
        Serial.println("Motor off");
    }

}

#if BLUETOOTH_HID
#if CONFIG_BT_HID_HOST_ENABLED
#pragma warning "HID Host enabled"
#endif    
#if CONFIG_BT_BLE_ENABLED
#pragma warning "BLE enabled"
#endif

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
    }
    break;
    case ESP_HIDH_CLOSE_EVENT: {
    }
    }
}


void esp_hid_scan_results_free(esp_hid_scan_result_t *results)
{
    esp_hid_scan_result_t *r = NULL;
    while (results) {
        r = results;
        results = results->next;
        if (r->name != NULL) {
            free((char *)r->name);
        }
        free(r);
    }
}

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
                if (strncmp(r->name, remote_device_name, strlen(remote_device_name)) == 0) {
                    break;
                }
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }

#if CONFIG_BT_HID_HOST_ENABLED
        if (cr && strncmp(cr->name, remote_device_name, strlen(remote_device_name)) == 0) {
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
#else
        if (cr) {
            //open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
#endif // CONFIG_BT_HID_HOST_ENABLED
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}
#endif

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
    shiftUpButton.attachClick(handleShiftUpClick);
    shiftDownButton.attachClick(handleShiftDownClick);
    xTaskCreatePinnedToCore(guiTask, "GUI Task", 4096*3, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(commTask, "COMM Task", 4096, NULL, 3, NULL, 1);

#if BLUETOOTH_HID
    esp_bluedroid_enable(); 
    esp_bluedroid_init();
 esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    esp_hidh_init(&config);
    esp_bt_hid_host_init();
    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);

#endif    
}

void loop() {
    delay(1);
    controller.setMotorConnected(motorc.isConnected());
    motorc.setDesiredTorqueLeft(controller.getCalculatedTorqueLeft());
    motorc.setDesiredTorqueRight(controller.getCalculatedTorqueRight());
    shiftDownButton.tick();
    shiftUpButton.tick();
}