#include <stdio.h>
#include <math.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_wifi.h>  // Added for WiFi functions
#include <nvs_flash.h> // Added for NVS

#include <lvgl.h>
#include <esp_lvgl_port.h>

#include "lcd.h"
#include "touch.h"
#include "mqtt_relay_client.h"

static const char *TAG = "water_control";
// UI objects
static lv_obj_t *toggle_btn;
static lv_obj_t *btn_label;
static lv_obj_t *timer_label;
static lv_timer_t *countdown_timer = NULL;

// WiFi status UI elements
static lv_obj_t *wifi_panel;
static lv_obj_t *wifi_ssid_label;
static lv_obj_t *wifi_strength_bars[4]; // 4 bars for signal strength
static lv_timer_t *wifi_update_timer = NULL;

// Timer variables
static int seconds_remaining = 300; // 5 minutes = 300 seconds
static bool timer_running = false;

// Forward declarations
static void toggle_event_cb(lv_event_t *e);
static void countdown_timer_cb(lv_timer_t *timer);
static void update_timer_display();
static void start_countdown();
static void stop_countdown();
static void create_wifi_status_panel(lv_obj_t *parent);
static void update_wifi_status();
static void wifi_update_timer_cb(lv_timer_t *timer);

// Event handler for toggle button
static void toggle_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    
    if (code == LV_EVENT_VALUE_CHANGED) {
        bool is_checked = lv_obj_has_state(obj, LV_STATE_CHECKED);
        
        if (is_checked) {
            // Button toggled ON - start timer
            ESP_LOGI(TAG, "Water turned ON");
            
            // Change button label to "Turn Water Off"
            lv_label_set_text(btn_label, "Turn Water Off");
            
            // Set relay 1 ON via MQTT
            mqtt_publish_relay_state(1, true);
            
            // Start the countdown
            start_countdown();
        } else {
            // Button toggled OFF - stop timer
            ESP_LOGI(TAG, "Water turned OFF");
            
            // Change button label back to "Turn Water On"
            lv_label_set_text(btn_label, "Turn Water On");
            
            // Set relay 1 OFF via MQTT
            mqtt_publish_relay_state(1, false);
            
            // Stop the countdown
            stop_countdown();
        }
    }
}

// Timer callback function
static void countdown_timer_cb(lv_timer_t *timer) {
    if (!timer_running) return;
    
    seconds_remaining--;
    update_timer_display();
    
    if (seconds_remaining <= 0) {
        // Time's up - turn off the water
        ESP_LOGI(TAG, "Timer expired, turning water OFF");
        
        if (lvgl_port_lock(0)) {
            lv_obj_clear_state(toggle_btn, LV_STATE_CHECKED);
            // Also update the button label when timer expires
            lv_label_set_text(btn_label, "Turn Water On");
            lvgl_port_unlock();
        }
        
        // Set relay 1 OFF via MQTT
        mqtt_publish_relay_state(1, false);
        
        // Stop the timer
        stop_countdown();
    }
}

// Update the timer display
static void update_timer_display() {
    char time_str[16];
    int minutes = seconds_remaining / 60;
    int seconds = seconds_remaining % 60;
    
    sprintf(time_str, "%02d:%02d", minutes, seconds);
    
    if (lvgl_port_lock(0)) {
        lv_label_set_text(timer_label, time_str);
        lvgl_port_unlock();
    }
}

// Start the countdown timer
static void start_countdown() {
    seconds_remaining = 300; // Reset to 5 minutes
    timer_running = true;
    
    update_timer_display();
    
    if (countdown_timer == NULL) {
        countdown_timer = lv_timer_create(countdown_timer_cb, 1000, NULL);
    } else {
        lv_timer_resume(countdown_timer);
    }
}

// Stop the countdown timer
static void stop_countdown() {
    timer_running = false;
    seconds_remaining = 300; // Reset to 5 minutes
    
    if (countdown_timer != NULL) {
        lv_timer_pause(countdown_timer);
    }
    
    update_timer_display();
}

// Create WiFi status panel at the bottom left
static void create_wifi_status_panel(lv_obj_t *parent) {
    // Create a panel for WiFi status info
    wifi_panel = lv_obj_create(parent);
    lv_obj_remove_style_all(wifi_panel);
    lv_obj_set_size(wifi_panel, 170, 60);
    lv_obj_align(wifi_panel, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    
    // Create SSID label
    wifi_ssid_label = lv_label_create(wifi_panel);
    lv_obj_set_style_text_color(wifi_ssid_label, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(wifi_ssid_label, "WiFi: ---");
    lv_obj_align(wifi_ssid_label, LV_ALIGN_TOP_LEFT, 0, 0);
    
    // Create signal strength bars
    for (int i = 0; i < 4; i++) {
        wifi_strength_bars[i] = lv_obj_create(wifi_panel);
        lv_obj_set_size(wifi_strength_bars[i], 8, 5 + (i+1) * 3);
        lv_obj_align(wifi_strength_bars[i], LV_ALIGN_BOTTOM_LEFT, 10 + i*12, 0);
        lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0x888888), LV_PART_MAIN); // Gray (inactive)
        lv_obj_set_style_border_width(wifi_strength_bars[i], 0, LV_PART_MAIN);
        lv_obj_set_style_radius(wifi_strength_bars[i], 1, LV_PART_MAIN);
    }
    
    // Start periodic updates
    wifi_update_timer = lv_timer_create(wifi_update_timer_cb, 5000, NULL); // Update every 5 seconds
    
    // Initial update
    update_wifi_status();
}

// Update WiFi status information
static void update_wifi_status() {
    if (!lvgl_port_lock(0)) {
        return;
    }
    
    // Get WiFi status
    wifi_ap_record_t ap_info;
    bool is_connected = false;
    int8_t rssi = -100; // Default poor signal
    char ssid[33] = {0}; // Max SSID length is 32 bytes + null terminator
    
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        is_connected = true;
        rssi = ap_info.rssi;
        memcpy(ssid, (char *)ap_info.ssid, sizeof(ap_info.ssid));
    }
    
    // Update SSID label
    if (is_connected) {
        char ssid_text[50];
        snprintf(ssid_text, sizeof(ssid_text), "WiFi: %s", ssid);
        lv_label_set_text(wifi_ssid_label, ssid_text);
    } else {
        lv_label_set_text(wifi_ssid_label, "WiFi: Not Connected");
    }
    
    // Update signal strength bars
    if (is_connected) {
        // RSSI ranges typically from -30 (excellent) to -90 (unusable)
        // Convert to 0-4 bars
        int bars = 0;
        if (rssi >= -55) bars = 4;      // Excellent: -55 to -30 dBm
        else if (rssi >= -67) bars = 3; // Good: -67 to -56 dBm
        else if (rssi >= -77) bars = 2; // Fair: -77 to -68 dBm
        else if (rssi >= -87) bars = 1; // Poor: -87 to -78 dBm
                                        // No bars: < -88 dBm
        
        for (int i = 0; i < 4; i++) {
            // Set color based on active status
            if (i < bars) {
                // Active bars - green to yellow based on strength
                if (bars >= 3) {
                    lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0x00FF00), LV_PART_MAIN); // Green for good signal
                } else if (bars >= 2) {
                    lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0xFFFF00), LV_PART_MAIN); // Yellow for medium signal
                } else {
                    lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0xFF8800), LV_PART_MAIN); // Orange for weak signal
                }
            } else {
                lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0x888888), LV_PART_MAIN); // Gray for inactive
            }
        }
    } else {
        // No connection - all bars gray
        for (int i = 0; i < 4; i++) {
            lv_obj_set_style_bg_color(wifi_strength_bars[i], lv_color_hex(0x888888), LV_PART_MAIN);
        }
    }
    
    lvgl_port_unlock();
}

// WiFi status update timer callback
static void wifi_update_timer_cb(lv_timer_t *timer) {
    update_wifi_status();
}

static esp_err_t app_lvgl_main(void) {
    lvgl_port_lock(0);
    
    // Get active screen with NULL check
    lv_obj_t *scr = lv_scr_act();
    if (scr == NULL) {
        ESP_LOGI(TAG, "No active screen found, creating a new one");
        scr = lv_obj_create(NULL);
        lv_scr_load(scr);
    }
    
    // Clear screen - set background to black
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    //lv_obj_set_style_bg_color(scr, lv_color_hex(0x2222FF), LV_PART_MAIN); // Blue background

    // Create toggle button
    toggle_btn = lv_btn_create(scr);
    lv_obj_add_flag(toggle_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_size(toggle_btn, 160, 60);
    lv_obj_align(toggle_btn, LV_ALIGN_TOP_LEFT, 10, 10);
    
    // Set button styles for both states
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x0000FF), LV_PART_MAIN); // Blue background for OFF state
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0xFF0000), LV_STATE_CHECKED); // Red background for ON state
    
    // Create label on the button
    btn_label = lv_label_create(toggle_btn);
    lv_label_set_text(btn_label, "Turn Water On");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), LV_PART_MAIN); // White text for OFF state
    lv_obj_set_style_text_color(btn_label, lv_color_black(), LV_STATE_CHECKED); // Black text for ON state
    lv_obj_center(btn_label);
    
    // Add event handler for the toggle button
    lv_obj_add_event_cb(toggle_btn, toggle_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Create the timer display label (with doubled size)
    timer_label = lv_label_create(scr);
    lv_obj_set_style_text_font(timer_label, &lv_font_montserrat_48, LV_PART_MAIN); // Largest standard font
    lv_obj_set_style_text_color(timer_label, lv_color_white(), LV_PART_MAIN);
    // No transform scaling
    lv_obj_align(timer_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(timer_label, "05:00");
    
    // Create WiFi status panel
    create_wifi_status_panel(scr);
    
    lvgl_port_unlock();
    
    return ESP_OK;
}

// Callback function to handle state changes from MQTT
static void mqtt_state_callback(uint8_t relay_num, bool state) {
    ESP_LOGI(TAG, "Received MQTT state change: relay %d -> %s", 
             relay_num, state ? "ON" : "OFF");
    
    // Update UI to match the MQTT state
    if (lvgl_port_lock(0)) {
        if (state) {
            // Turn ON
            lv_obj_add_state(toggle_btn, LV_STATE_CHECKED);
            lv_label_set_text(btn_label, "Turn Water Off");
            
            // Start countdown if not running
            if (!timer_running) {
                start_countdown();
            }
        } else {
            // Turn OFF
            lv_obj_clear_state(toggle_btn, LV_STATE_CHECKED);
            lv_label_set_text(btn_label, "Turn Water On");
            
            // Stop countdown if running
            if (timer_running) {
                stop_countdown();
            }
        }
        lvgl_port_unlock();
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    static const char *LCD_TAG = "lcd";
    
    // Initialize display
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    esp_lcd_panel_handle_t lcd_panel = NULL;
    ESP_ERROR_CHECK(app_lcd_init(&lcd_io, &lcd_panel));
    
    // Initialize backlight control but keep it OFF initially
    ESP_ERROR_CHECK(lcd_display_brightness_init());
    
    // Initialize LVGL
    lv_display_t *disp = app_lvgl_init(lcd_io, lcd_panel);

    // Initialize touch
    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(app_touch_init(&tp));

    // Register touch with LVGL port
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };
    lvgl_port_add_touch(&touch_cfg);
    
    // Initialize MQTT client
    mqtt_init();
    mqtt_register_state_change_callback(mqtt_state_callback);
    
    // Initialize LVGL UI (with display still off)
    ESP_ERROR_CHECK(app_lvgl_main());
    
    // Force a display refresh to ensure UI is fully drawn before turning on backlight
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Now turn on the backlight at full brightness
    ESP_LOGI(LCD_TAG, "Turning on backlight to 100%%");
    ESP_ERROR_CHECK(lcd_display_brightness_set(100));
}
