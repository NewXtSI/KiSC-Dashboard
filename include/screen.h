#pragma once
#include <Arduino.h>
#include <lvgl.h>

#define DASH_SYMBOL_INDICATOR_LEFT  _BV(0)
#define DASH_SYMBOL_INDICATOR_RIGHT _BV(1)
#define DASH_SYMBOL_LAUNCH_CONTROL _BV(2)
#define DASH_SYMBOL_PARKING_BRAKE _BV(3)
#define DASH_SYMBOL_LOW_BEAM _BV(4)
#define DASH_SYMBOL_CHECK_ENGINE _BV(5)
#define DASH_SYMBOL_ESP _BV(6)
#define DASH_SYMBOL_FUEL_POWER _BV(7)
#define DASH_SYMBOL_HIGH_BEAM _BV(8)



class Screen {
public:
    Screen();
    virtual ~Screen();
    virtual void create();
    lv_obj_t* getScreen() { return screen; }
    virtual void update() = 0;
    virtual void activate();
    void setSymbols(uint32_t uiSymbols) { symbols = uiSymbols; }
    uint32_t getSymbols() { return symbols; }
    bool isScreenActivated() { return isActivated; }
private:
    uint32_t symbols;
    virtual void init();
    bool    isActivated;
protected:
    lv_obj_t* screen;
};
