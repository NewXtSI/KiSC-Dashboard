#define DISPLAY_IPS

#include <Arduino.h>
#include <lvgl.h>
#include "hardware.h"


#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>

#include <display/lv_display_private.h>
#include <misc/lv_timer_private.h>
#include <indev/lv_indev_private.h>
#include <memory>

#include "globals.h"
#define ESP32DEBUGGING
#include <ESP32Logger.h>

std::pair<void*, void*> draw_buffers = {nullptr, nullptr};
esp_lcd_panel_handle_t panel_handle = nullptr;

#ifdef __cplusplus
extern "C" {
#endif
void my_print(lv_log_level_t level, const char * buf) {
//    Serial.printf("%s\n", buf);
}
#ifdef __cplusplus
}
#endif

void draw_image(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end, uint16_t* data)
{
//    Serial.printf("draw_image: x_start:%d, x_end:%d, y_start:%d, y_end:%d\n", x_start, x_end, y_start, y_end);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end + 1, y_end + 1, data));
}

bool st7789_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

lv_display_t *lvgl_lcd_init2(uint32_t hor_res, uint32_t ver_res) {

    lv_display_t *display;
#if LV_USE_LOG
    lv_log_register_print_cb([](lv_log_level_t level, const char *message) {
    LV_UNUSED(level);
//            Serial.println(message);
            // Serial.flush();
        });
#endif        
    display = lv_display_create(hor_res, ver_res);        
    delay(50);
    // Create bufferydoos
    // It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    // https://github.com/lvgl/lvgl/blob/5ce363464fc81dec5378fc02a341b35786f0946b/docs/integration/driver/display/lcd_stm32_guide.rst
    // https://github.com/espressif/esp-idf/blob/b3f7e2c8a4d354df8ef8558ea7caddc07283a57b/examples/peripherals/lcd/i80_controller/main/i80_controller_example_main.c#L453
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/mm_sync.html#memory-allocation-helper
    
    const auto buffer_size = hor_res * ver_res * lv_color_format_get_size(lv_display_get_color_format(display));

    draw_buffers.first = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!draw_buffers.first)
    {
        Serial.printf("display draw buffer malloc failed\n");
//        LV_LOG_ERROR("display draw buffer malloc failed");
        return nullptr;
    }
#if 0
    draw_buffers.second = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!draw_buffers.second)
    {
        Serial.printf("display buffer malloc failed\n");
//        LV_LOG_ERROR("display buffer malloc failed");
        lv_free(draw_buffers.first);
        return nullptr;
    }
#endif
    // Create SPI bus
    const spi_bus_config_t spi_bus_config = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 7,
        .flags = 0,
        .intr_flags = 0};
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "spi_bus_config: mosi_io_num:%d, miso_io_num:%d, sclk_io_num:%d, quadwp_io_num:%d, quadhd_io_num:%d, max_transfer_sz:%d, flags:0x%08x, intr_flags:0x%04x\n", spi_bus_config.mosi_io_num, spi_bus_config.miso_io_num, spi_bus_config.sclk_io_num, spi_bus_config.quadwp_io_num, spi_bus_config.quadhd_io_num, spi_bus_config.max_transfer_sz, spi_bus_config.flags, spi_bus_config.intr_flags);
//    log_d("spi_bus_config: mosi_io_num:%d, miso_io_num:%d, sclk_io_num:%d, quadwp_io_num:%d, quadhd_io_num:%d, max_transfer_sz:%d, flags:0x%08x, intr_flags:0x%04x", spi_bus_config.mosi_io_num, spi_bus_config.miso_io_num, spi_bus_config.sclk_io_num, spi_bus_config.quadwp_io_num, spi_bus_config.quadhd_io_num, spi_bus_config.max_transfer_sz, spi_bus_config.flags, spi_bus_config.intr_flags);
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));

    // Attach the LCD controller to the SPI bus
    const esp_lcd_panel_io_spi_config_t io_spi_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 3,
        .pclk_hz = 24000000,
        .trans_queue_depth = 10,
        .on_color_trans_done = st7789_color_trans_done,
        .user_ctx = display,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .dc_as_cmd_phase = false,
            .dc_low_on_data = false,
            .octal_mode = false,
            .lsb_first = false}};
    
    //log_d("io_spi_config: cs_gpio_num:%d, dc_gpio_num:%d, spi_mode:%d, pclk_hz:%d, trans_queue_depth:%d, user_ctx:0x%08x, on_color_trans_done:0x%08x, lcd_cmd_bits:%d, lcd_param_bits:%d, flags:{dc_as_cmd_phase:%d, dc_low_on_data:%d, octal_mode:%d, lsb_first:%d}", io_spi_config.cs_gpio_num, io_spi_config.dc_gpio_num, io_spi_config.spi_mode, io_spi_config.pclk_hz, io_spi_config.trans_queue_depth, io_spi_config.user_ctx, io_spi_config.on_color_trans_done, io_spi_config.lcd_cmd_bits, io_spi_config.lcd_param_bits, io_spi_config.flags.dc_as_cmd_phase, io_spi_config.flags.dc_low_on_data, io_spi_config.flags.octal_mode, io_spi_config.flags.lsb_first);
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "io_spi_config: cs_gpio_num:%d, dc_gpio_num:%d, spi_mode:%d, pclk_hz:%d, trans_queue_depth:%d, user_ctx:0x%08x, on_color_trans_done:0x%08x, lcd_cmd_bits:%d, lcd_param_bits:%d, flags:{dc_as_cmd_phase:%d, dc_low_on_data:%d, octal_mode:%d, lsb_first:%d}\n", io_spi_config.cs_gpio_num, io_spi_config.dc_gpio_num, io_spi_config.spi_mode, io_spi_config.pclk_hz, io_spi_config.trans_queue_depth, io_spi_config.user_ctx, io_spi_config.on_color_trans_done, io_spi_config.lcd_cmd_bits, io_spi_config.lcd_param_bits, io_spi_config.flags.dc_as_cmd_phase, io_spi_config.flags.dc_low_on_data, io_spi_config.flags.octal_mode, io_spi_config.flags.lsb_first);
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_spi_config, &io_handle));

    // Create st7789 panel handle
    const esp_lcd_panel_dev_config_t panel_dev_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = false},
        .vendor_config = NULL};
    //log_d("panel_dev_config: reset_gpio_num:%d, color_space:%d, bits_per_pixel:%d, flags:{reset_active_high:%d}, vendor_config:0x%08x", panel_dev_config.reset_gpio_num, panel_dev_config.color_space, panel_dev_config.bits_per_pixel, panel_dev_config.flags.reset_active_high, panel_dev_config.vendor_config);
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "panel_dev_config: reset_gpio_num:%d, color_space:%d, bits_per_pixel:%d, flags:{reset_active_high:%d}, vendor_config:0x%08x\n", panel_dev_config.reset_gpio_num, panel_dev_config.color_space, panel_dev_config.bits_per_pixel, panel_dev_config.flags.reset_active_high, panel_dev_config.vendor_config);

    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_new_panel_st7789\n");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_dev_config, &panel_handle));
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_reset\n");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_init\n");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // 4. Adding manufacturer specific initialization 

    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_invert_color\n");
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_swap_xy\n");
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_mirror\n");

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_set_gap\n");
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 40, 53));    
//    esp_lcd_panel_set_gap(panel_handle,40,52);
#ifdef DISPLAY_IPS
    // If LCD is IPS invert the colors
//    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
#if (DISPLAY_SWAP_XY)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, DISPLAY_SWAP_XY));
#endif
#if (DISPLAY_MIRROR_X || DISPLAY_MIRROR_Y)
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
#endif
#if (DISPLAY_GAP_X || DISPLAY_GAP_Y)
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, DISPLAY_GAP_X, DISPLAY_GAP_Y));
#endif
    // Turn display on
    DBGCHK(Verbose, SERIAL_DEBUG_GUI, "esp_lcd_panel_disp_on_off\n");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));    


    lv_display_set_buffers(display, draw_buffers.first, nullptr, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
//    lv_display_set_buffers(display, draw_buffers.first, draw_buffers.second, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    const auto flush_callback = [](lv_display_t *disp, const lv_area_t *area, uint8_t *pix_map)
    {
        lv_draw_sw_rgb565_swap(pix_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
        draw_image(area->x1, area->x2, area->y1, area->y2, reinterpret_cast<uint16_t*>(pix_map));
    };

    lv_display_set_flush_cb(display, flush_callback); 


    return display;   
}











#define DISP_ROT  0     // 0°

#undef LCD_H_RES
#undef LCD_V_RES

#if DISP_ROT == 0 
#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_ROT LV_DISPLAY_ROTATION_0
//#define DISPLAY_GAP_X 52
//#define DISPLAY_GAP_Y 40
#elif DISP_ROT == 180
#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_ROT LV_DISPLAY_ROTATION_180
#endif

bool st7789_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *display = (lv_display_t*)user_ctx;
//    Serial.printf("color trans done\n");
    lv_display_flush_ready(display);
    return false;
}

void st7789_lv_flush(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)(display->user_data);
    uint32_t pixels = lv_area_get_size(area);
    uint16_t *p = (uint16_t *)px_map;
    while (pixels--)
    {
        *p = (uint16_t)((*p >> 8) | (*p << 8));
        p++;
    }
    Serial.printf("flush: x1:%d, y1:%d, x2:%d, y2:%d\n", area->x1, area->y1, area->x2, area->y2);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map));
};

lv_display_t *lvgl_lcd_init(uint32_t hor_res, uint32_t ver_res)
{
    lv_display_t *display = lv_display_create(hor_res, ver_res);
    log_v("display:0x%08x", display);
    //  Create drawBuffer
    uint32_t drawBufferSize = sizeof(lv_color_t) * (240 * 240);
    void *drawBuffer = heap_caps_malloc(drawBufferSize, (MALLOC_CAP_8BIT));
    lv_display_set_buffers(display, drawBuffer, NULL, drawBufferSize, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Hardware rotation is supported
//    display->sw_rotate = 0;
    display->rotation = LCD_ROT;

    // Create SPI bus
    const spi_bus_config_t spi_bus_config = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 7,
        .flags = 0,
        .intr_flags = 0};
    log_d("spi_bus_config: mosi_io_num:%d, miso_io_num:%d, sclk_io_num:%d, quadwp_io_num:%d, quadhd_io_num:%d, max_transfer_sz:%d, flags:0x%08x, intr_flags:0x%04x", spi_bus_config.mosi_io_num, spi_bus_config.miso_io_num, spi_bus_config.sclk_io_num, spi_bus_config.quadwp_io_num, spi_bus_config.quadhd_io_num, spi_bus_config.max_transfer_sz, spi_bus_config.flags, spi_bus_config.intr_flags);
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));

    // Attach the LCD controller to the SPI bus
    const esp_lcd_panel_io_spi_config_t io_spi_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 3,
        .pclk_hz = 24000000,
        .trans_queue_depth = 10,
        .on_color_trans_done = st7789_color_trans_done,
        .user_ctx = display,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .dc_as_cmd_phase = false,
            .dc_low_on_data = false,
            .octal_mode = false,
            .lsb_first = false}};
    log_d("io_spi_config: cs_gpio_num:%d, dc_gpio_num:%d, spi_mode:%d, pclk_hz:%d, trans_queue_depth:%d, user_ctx:0x%08x, on_color_trans_done:0x%08x, lcd_cmd_bits:%d, lcd_param_bits:%d, flags:{dc_as_cmd_phase:%d, dc_low_on_data:%d, octal_mode:%d, lsb_first:%d}", io_spi_config.cs_gpio_num, io_spi_config.dc_gpio_num, io_spi_config.spi_mode, io_spi_config.pclk_hz, io_spi_config.trans_queue_depth, io_spi_config.user_ctx, io_spi_config.on_color_trans_done, io_spi_config.lcd_cmd_bits, io_spi_config.lcd_param_bits, io_spi_config.flags.dc_as_cmd_phase, io_spi_config.flags.dc_low_on_data, io_spi_config.flags.octal_mode, io_spi_config.flags.lsb_first);
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_spi_config, &io_handle));

    // Create st7789 panel handle
    const esp_lcd_panel_dev_config_t panel_dev_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = false},
        .vendor_config = NULL};
    log_d("panel_dev_config: reset_gpio_num:%d, color_space:%d, bits_per_pixel:%d, flags:{reset_active_high:%d}, vendor_config:0x%08x", panel_dev_config.reset_gpio_num, panel_dev_config.color_space, panel_dev_config.bits_per_pixel, panel_dev_config.flags.reset_active_high, panel_dev_config.vendor_config);
    esp_lcd_panel_handle_t panel_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_dev_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
//    esp_lcd_panel_set_gap(panel_handle,40,52);
#ifdef DISPLAY_IPS
    // If LCD is IPS invert the colors
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
#if (DISPLAY_SWAP_XY)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, DISPLAY_SWAP_XY));
#endif
#if (DISPLAY_MIRROR_X || DISPLAY_MIRROR_Y)
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
#endif
#if (DISPLAY_GAP_X || DISPLAY_GAP_Y)
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, DISPLAY_GAP_X, DISPLAY_GAP_Y));
#endif
    // Turn display on
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    display->user_data = panel_handle;
    display->flush_cb = st7789_lv_flush;

    return display;
}


#if 0
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lvgl.h>
#include "hardware.h"

// Erstellen des Displayobjekts
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;

public:
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI3_HOST; // oder HSPI_HOST, je nach Setup
            cfg.spi_mode = 3;
            cfg.freq_write = 40000000; // 40 MHz
            cfg.freq_read = 16000000;  // 16 MHz
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = 1;
            cfg.pin_sclk = PIN_LCD_SCLK; // Pin für SCLK (anpassen an dein Board)
            cfg.pin_mosi = GPIO_NUM_45; // Pin für MOSI (anpassen an dein Board)
            cfg.pin_miso = -1; // MISO nicht verwendet
            cfg.pin_dc = GPIO_NUM_41;   // Pin für DC
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = PIN_LCD_CS;  // Pin für CS (anpassen an dein Setup)
            cfg.pin_rst = PIN_LCD_RST; // Pin für RST (anpassen an dein Setup)
            cfg.pin_busy = -1;
            cfg.memory_width = 240;  // Breite des Displays
            cfg.memory_height = 240; // Höhe des Displays
            cfg.panel_width = 240;
            cfg.panel_height = 240;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 16;
            cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = true;       // ST7789 erfordert oft Invertierung der Farben
            cfg.rgb_order = false;   // RGB Farb-Reihenfolge
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};

// Instanz des Displays erstellen
LGFX tft;
#define LV_HOR_RES_MAX 240
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);

void lvgl_setup()
{
    // Initialisierung von LovyanGFX und LVGL
    tft.init();
    lv_init();

    static lv_draw_buf_t draw_buf;
    static lv_color_t buf[LV_HOR_RES_MAX * 10];
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    static lv_disp_drv_t disp_drv;
    lv_draw_buf_init(&disp_drv);
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

// Display-Flush Funktion
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);
    tft.writePixels((lgfx::rgb565_t *)&color_p->full, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
    tft.endWrite();
    lv_disp_flush_ready(disp);
}
#endif
