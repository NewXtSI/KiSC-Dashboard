#pragma once
#include "screen.h"

class BootScreen : public Screen {
public:
    BootScreen();
    void create() override;
private:
    void init() override;
    void update() override;
};
