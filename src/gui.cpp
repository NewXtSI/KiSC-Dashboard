#include <Arduino.h>
#include "gui.h"
#include <lvgl.h>
#include "hardware.h"
#include "bootscreen.h"
#include "dash.h"

#include "globals.h"
#define ESP32DEBUGGING
#include <ESP32Logger.h>

lv_display_t *lvgl_lcd_init(uint32_t hor_res, uint32_t ver_res);
lv_display_t *lvgl_lcd_init2(uint32_t hor_res, uint32_t ver_res);
lv_display_t *lvgl_lcd_init_espi(uint32_t hor_res, uint32_t ver_res);
lv_display_t *lvgl_lcd_init_lgfx(uint32_t hor_res, uint32_t ver_res);

GUI::GUI(Motor& motor, Controller& controller) : motor(motor), controller(controller) {
    activeScreen = nullptr; // Initialize the active screen to nullptr

}

#define LV_TICK_PERIOD_MS 2
lv_display_t *display;

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
//    lv_indev_read(indev);

}


void GUI::init() {
    // Initialization code
    lv_init();
    display = lvgl_lcd_init2(240, 135);
    if (display == nullptr) {
         DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Display initialization failed!!!!");
    }
    lv_obj_clean(lv_scr_act());
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hallo Welt q!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void GUI::create() {
    Serial.printf("Creating GUI\n");
    activeScreen = new BootScreen();
    if (activeScreen != nullptr) {
         DBGCHK(Verbose, SERIAL_DEBUG_GUI, "BootScreen created");
         DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Creating BootScreen");
        activeScreen->create();
         DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Activating BootScreen");
        activeScreen->activate();
    } else {
         DBGCHK(Verbose, SERIAL_DEBUG_GUI, "BootScreen creation failed");
    }
       /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000);    
     DBGCHK(Verbose, SERIAL_DEBUG_GUI, "GUI created");

    // Create and start a timer to switch to the dashboard screen after 5 seconds
    const esp_timer_create_args_t screenSwitchTimerArgs = {
        .callback = &GUI::screenSwitchCallback,
        .arg = this,
        .name = "screenSwitchTimer"
    };
    esp_timer_create(&screenSwitchTimerArgs, &screenSwitchTimer);
    esp_timer_start_once(screenSwitchTimer, 2000000); // 5 seconds in microseconds
}

void GUI::switchScreen(int screenId) {
 DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Switching screen to %d", screenId);    
    switch (screenId) {
        case 0:
//            activeScreen = &bootScreen;
            break;
        case 1:
            activeScreen = new Dash();
//            activeScreen->create();
            break;
    }
    activeScreen->activate();
    this->screenId = screenId;
}

void GUI::screenSwitchCallback(void* arg) {
    GUI* gui = (GUI*)arg;
    gui->switchScreen(1);
}

void GUI::update() {
    if (screenId == 1) {
        ((Dash*)(activeScreen))->setBlinkHigh(controller.getIndicatorOn());
        if (!motor.isConnected()) {
            ((Dash*)(activeScreen))->setSpeed(0);
            ((Dash*)(activeScreen))->setThrottle(controller.getCompensatedThrottle());
            ((Dash*)(activeScreen))->setTemperature(0);
            ((Dash*)(activeScreen))->setBattery(0);

            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_CHECK_ENGINE);
        } else {
            ((Dash*)(activeScreen))->setSpeed(motor.getLeftSpeed());
            ((Dash*)(activeScreen))->setThrottle(controller.getCompensatedThrottle());
            ((Dash*)(activeScreen))->setTemperature(motor.getTemperature());
            ((Dash*)(activeScreen))->setBattery(motor.getVoltage());

            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_CHECK_ENGINE);
        }
        ((Dash*)(activeScreen))->setPower(controller.getRPM());

        ((Dash*)(activeScreen))->setDriveMode(controller.getDriveMode());

/*        if (controller.getIndicatorOn()) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_INDICATOR_LEFT);
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_INDICATOR_RIGHT);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_INDICATOR_LEFT);
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_INDICATOR_RIGHT);
        }*/
        if (controller.getLightStates().lowBeam) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_LOW_BEAM);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_LOW_BEAM);
        }
        if (controller.getLightStates().highBeam) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_HIGH_BEAM);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_HIGH_BEAM);
        }
        if (controller.getDriveMode() == PARKING) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_PARKING_BRAKE);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_PARKING_BRAKE);
        }
        if (controller.getESPWorking()) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_ESP);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_ESP);
        }
        if (controller.getLaunchControlActive()) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_LAUNCH_CONTROL);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_LAUNCH_CONTROL);
        }
        if (motor.isCharging()) {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() | DASH_SYMBOL_CHARGING);
        } else {
            ((Dash*)(activeScreen))->setSymbols(((Dash*)(activeScreen))->getSymbols() & ~DASH_SYMBOL_CHARGING);
        }
    }
    activeScreen->update();
    lv_task_handler();

    //if activescreen is of type dash, update the GUI with the current state of motor and controller




    // Update the GUI with the current state of motor and controller
//    int motorSpeed = motor.getLeftSpeed(); // Example usage
//    int controllerSteering = controller.getSteering(); // Example usage
    // Update GUI elements with motorSpeed and controllerSteering
}

