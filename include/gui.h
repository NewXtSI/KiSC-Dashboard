#ifndef GUI_H
#define GUI_H

#include "motor.h"
#include "controller.h"
#include "screen.h"

// A simple GUI class, that encapsulates the GUI logic for an LVGL9 based GUI
class GUI {
public:
    GUI(Motor& motor, Controller& controller);
    void init();
    void create();
    void update();
    void switchScreen(int screenId);
private:
    int         screenId;
    esp_timer_handle_t screenSwitchTimer;
    Screen* activeScreen;
    Motor& motor;
    Controller& controller;
    static void screenSwitchCallback(void* arg);
};

#endif