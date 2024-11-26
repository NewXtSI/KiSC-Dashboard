#include "screen.h"

Screen::Screen() {
    init();
    create();
    isActivated = false;
}

Screen::~Screen() {
    lv_obj_clean(screen);
    lv_obj_del(screen);
}

void Screen::init() {
    screen = lv_obj_create(NULL);
}

void Screen::update() {
    lv_task_handler();
}

void  Screen::activate() { 
    lv_scr_load(screen);
    isActivated = true;
}

void Screen::create() {

}