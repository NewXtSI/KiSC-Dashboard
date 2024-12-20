#include <Arduino.h>
#include <WiFi.h>

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


void setup() {
  // put your setup code here, to run once:
    esp_task_wdt_init(60, true);
    esp_task_wdt_deinit();
    Serial.begin(115200);
    delay(500);
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
}

void loop() {
    delay(1);
}