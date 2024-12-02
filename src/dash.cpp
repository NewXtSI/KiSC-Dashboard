#include "dash.h"
//#include "../fonts/Includes/Arcade_54.c"
#include "../fonts/Includes/Federals_Chrome_Ital_54.c"

LV_IMAGE_DECLARE(indicator_left);
LV_IMAGE_DECLARE(indicator_right);
LV_IMAGE_DECLARE(check_engine);
LV_IMAGE_DECLARE(esp);
LV_IMAGE_DECLARE(launch);
LV_IMAGE_DECLARE(parking);
LV_IMAGE_DECLARE(fuel_power);
LV_IMAGE_DECLARE(fuel_power_16);
LV_IMAGE_DECLARE(low_beam);

Dash::Dash() : Screen() {}

void Dash::init() {
    screen = lv_obj_create(NULL);
    setSymbols(0x000000);
}

//LV_FONT_DECLARE(Arcade_54)
LV_FONT_DECLARE(Federals_Chrome_Ital_54)

void Dash::create() {

    // Set the background to dark
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x020202), 0);
    battery_bar = lv_bar_create(screen);
    lv_obj_set_size(battery_bar, 15, 102);
    lv_obj_set_pos(battery_bar, 5, 5);
    lv_bar_set_range(battery_bar, 30000, 42000);    
    lv_bar_set_value(battery_bar, 40000, LV_ANIM_ON);
    lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x0C978E), LV_PART_INDICATOR);    

    temperature_bar = lv_bar_create(screen);
    lv_obj_set_size(temperature_bar, 15, 102);
    lv_obj_set_pos(temperature_bar, 22, 5);
    lv_bar_set_range(temperature_bar, 0, 80);    
    lv_bar_set_value(temperature_bar, 12, LV_ANIM_ON);
    lv_obj_set_style_bg_color(temperature_bar, lv_color_hex(0x0C978E), LV_PART_INDICATOR);    

    throttle_bar = lv_bar_create(screen);
    lv_obj_set_size(throttle_bar, 15, 102);
    lv_obj_set_pos(throttle_bar, 220, 5);
    lv_bar_set_range(throttle_bar, 0, 512);    
    lv_bar_set_value(throttle_bar, 50, LV_ANIM_ON);
    lv_obj_set_style_bg_color(throttle_bar, lv_color_hex(0x0C978E), LV_PART_INDICATOR);    

    power_bar = lv_bar_create(screen);
    lv_obj_set_size(power_bar, 15, 102);
    lv_obj_set_pos(power_bar, 203, 5);
    lv_bar_set_range(power_bar, 0, 1000);    
    lv_bar_set_value(power_bar, 50, LV_ANIM_ON);
    lv_obj_set_style_bg_color(power_bar, lv_color_hex(0x0C978E), LV_PART_INDICATOR);    

    speed_label = lv_label_create(screen);
     lv_label_set_text_fmt(speed_label, "%d", (int)speed);
//    lv_label_set_text(speed_label, "12");
    lv_obj_align(speed_label, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_text_font(speed_label, &Federals_Chrome_Ital_54, 0);
    lv_obj_set_style_text_color(speed_label, lv_color_hex(0x0C978E), 0);

    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, "km/h");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_text_color(label, lv_color_hex(0x0C978E), 0);

    drivemode_label = lv_label_create(screen);
    lv_label_set_text(drivemode_label, "Neutral");
    lv_obj_align(drivemode_label, LV_ALIGN_TOP_MID, 0, 120);
    lv_obj_set_style_text_color(drivemode_label, lv_color_hex(0x0C978E), 0);

    indicator_left_img = lv_img_create(screen);
    lv_img_set_src(indicator_left_img, &indicator_left);
    lv_obj_set_pos(indicator_left_img, 39, 2);
    lv_obj_set_size(indicator_left_img, 32, 32);
    lv_obj_add_flag(indicator_left_img, LV_OBJ_FLAG_HIDDEN);

    indicator_right_img = lv_img_create(screen);
    lv_img_set_src(indicator_right_img, &indicator_right);
    lv_obj_set_pos(indicator_right_img, 201-32, 2);
    lv_obj_add_flag(indicator_right_img, LV_OBJ_FLAG_HIDDEN);

    check_engine_img = lv_img_create(screen);
    lv_img_set_src(check_engine_img, &check_engine);
    lv_obj_set_pos(check_engine_img, 32, 135-2-32);
    lv_obj_add_flag(check_engine_img, LV_OBJ_FLAG_HIDDEN);

    esp_img = lv_img_create(screen);
    lv_img_set_src(esp_img, &esp);
    lv_obj_set_pos(esp_img, 70, 135-2-32);
    lv_obj_add_flag(esp_img, LV_OBJ_FLAG_HIDDEN);

    launch_img = lv_img_create(screen);
    lv_img_set_src(launch_img, &launch);
    lv_obj_set_pos(launch_img, 70, 2);
    lv_obj_add_flag(launch_img, LV_OBJ_FLAG_HIDDEN);

    park_img = lv_img_create(screen);
    lv_img_set_src(park_img, &parking);
    lv_obj_set_pos(park_img, 102, 2);
    lv_obj_add_flag(park_img, LV_OBJ_FLAG_HIDDEN);

    beam_img = lv_img_create(screen);
    lv_img_set_src(beam_img, &low_beam);
    lv_obj_set_pos(beam_img, 134, 2);
    lv_obj_add_flag(beam_img, LV_OBJ_FLAG_HIDDEN);

    fuel_power_img = lv_img_create(screen);
    lv_img_set_src(fuel_power_img, &fuel_power);
    lv_obj_set_pos(fuel_power_img, 201-32, 135-2-32);
    lv_obj_add_flag(fuel_power_img, LV_OBJ_FLAG_HIDDEN);

    fuel_power_16_img = lv_img_create(screen);
    lv_img_set_src(fuel_power_16_img, &fuel_power_16);
    lv_obj_set_pos(fuel_power_16_img, 5, 135-16);
}
 
void Dash::refreshStates() {
    lv_obj_invalidate(fuel_power_img);
    lv_obj_invalidate(fuel_power_16_img);
    lv_obj_invalidate(beam_img);
    lv_obj_invalidate(park_img);
    lv_obj_invalidate(launch_img);
    lv_obj_invalidate(esp_img);
    lv_obj_invalidate(check_engine_img);
    lv_obj_invalidate(indicator_right_img);
    lv_obj_invalidate(indicator_left_img);
}

void Dash::update() {
    if (isScreenActivated())  {
    // Check Symbols
    if (getSymbols() & DASH_SYMBOL_INDICATOR_LEFT) {
        // Show left indicator
        if (lv_obj_has_flag(indicator_left_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(indicator_left_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        if (!lv_obj_has_flag(indicator_left_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(indicator_left_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_INDICATOR_RIGHT) {
        // Show right indicator
        if (lv_obj_has_flag(indicator_right_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(indicator_right_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide right indicator
        if (!lv_obj_has_flag(indicator_right_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(indicator_right_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_CHECK_ENGINE) {
        // Show check engine
        if (lv_obj_has_flag(check_engine_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(check_engine_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else { 
        // Hide check engine
        if (!lv_obj_has_flag(check_engine_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(check_engine_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_ESP) {
        // Show ESP
        if (lv_obj_has_flag(esp_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(esp_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide ESP
        if (!lv_obj_has_flag(esp_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(esp_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_LAUNCH_CONTROL) {
        // Show launch control
        if (lv_obj_has_flag(launch_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(launch_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide launch control
        if (!lv_obj_has_flag(launch_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(launch_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_PARKING_BRAKE) {
        // Show parking brake
        if (lv_obj_has_flag(park_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(park_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide parking brake
        if (!lv_obj_has_flag(park_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(park_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_LOW_BEAM) {
        // Show low beam
        if (lv_obj_has_flag(beam_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(beam_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide low beam
        if (!lv_obj_has_flag(beam_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(beam_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (getSymbols() & DASH_SYMBOL_FUEL_POWER) {
        // Show fuel power
        if (lv_obj_has_flag(fuel_power_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_remove_flag(fuel_power_img, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // Hide fuel power
        if(!lv_obj_has_flag(fuel_power_img, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(fuel_power_img, LV_OBJ_FLAG_HIDDEN);
        }
    }
    refreshStates();

        if (speed_label != nullptr) {
            
            
            lv_label_set_text_fmt(speed_label, "%d", (int)speed);

            lv_bar_set_value(throttle_bar, throttle, LV_ANIM_ON);
            lv_bar_set_value(power_bar, power, LV_ANIM_ON);
            lv_bar_set_value(temperature_bar, (int16_t)(temperature), LV_ANIM_ON);

            if (getSymbols() & DASH_SYMBOL_CHARGING) {
                lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x0C970C), LV_PART_INDICATOR);
                lv_bar_set_value(battery_bar, (int16_t)battery*1000, LV_ANIM_ON);
                setSymbols(getSymbols() & ~DASH_SYMBOL_FUEL_POWER);
            } else {
                if (battery < 32) {
                    lv_bar_set_value(battery_bar, 31000, LV_ANIM_ON);
                    lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0xFF0000), LV_PART_INDICATOR);
                    setSymbols(getSymbols() | DASH_SYMBOL_FUEL_POWER);
                } else {
                   lv_bar_set_value(battery_bar, (int16_t)battery*1000, LV_ANIM_ON);
                    lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x0C978E), LV_PART_INDICATOR);
                    setSymbols(getSymbols() & ~DASH_SYMBOL_FUEL_POWER);
                }
            }

            switch (driveMode) {
                case MOTOR_OFF:
                    lv_label_set_text(drivemode_label, "Motor off");
                    break;
                case PARKING:
                    lv_label_set_text(drivemode_label, "Parking");
                    break;
                case NEUTRAL:
                    lv_label_set_text(drivemode_label, "Neutral");
                    break;
                case DRIVE:
                    lv_label_set_text(drivemode_label, "Drive");
                    break;
                case REVERSE:
                    lv_label_set_text(drivemode_label, "Reverse");
                    break;
            }
        }
    }

//    lv_task_handler();
}
