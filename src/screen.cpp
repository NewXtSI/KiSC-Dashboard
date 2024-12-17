#include "screen.h"
#include "globals.h"

Screen::Screen() {
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Screen::Screen()");
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "-> Init");
    init();
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "-> Create");
    create();
    isActivated = false;
}

Screen::~Screen() {
    lv_obj_clean(screen);
    lv_obj_del(screen);
}

void Screen::init() {
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Screen::init");
    screen = lv_obj_create(NULL);
}

void Screen::update() {
    lv_task_handler();
}

void  Screen::activate() { 
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Screen::activate()");
    lv_scr_load(screen);
    isActivated = true;
}

void Screen::create() {
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "Screen::create()");

}