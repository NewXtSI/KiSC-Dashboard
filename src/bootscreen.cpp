#include "bootscreen.h"

LV_IMAGE_DECLARE(amg);

// Display size is 240 x 135 pixels
BootScreen::BootScreen() : Screen() {}

static void anim_x_cb(void *var, int32_t v) {
    lv_obj_set_x((lv_obj_t *)var, v);
}


void BootScreen::create() {
/*    lv_obj_t *label = lv_label_create(screen);

    // Fill the screen with a dark anthracite color
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101030), 0);

    lv_label_set_text(label, "Hallo Boot!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
*/
    lv_obj_t *img = lv_img_create(screen);
    lv_img_set_src(img, &amg);

    lv_obj_t *ball = lv_obj_create(screen);
    lv_obj_set_size(ball, 20, 20);
    lv_obj_set_style_bg_color(ball, lv_color_hex(0x040404), 0);
    lv_obj_set_style_bg_opa(ball, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ball, LV_RADIUS_CIRCLE, 0); // Make it a circle

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ball);
    lv_anim_set_values(&a, 0, 240 - 20); // Move from left to right
    lv_anim_set_time(&a, 2000); // Duration of 5 seconds
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); // Repeat indefinitely
    lv_anim_start(&a);
}

void BootScreen::init() {
    screen = lv_obj_create(NULL);
}

void BootScreen::update() {
//    lv_task_handler();
}










