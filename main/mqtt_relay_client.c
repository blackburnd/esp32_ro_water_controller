#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "mqtt_relay_client.h"
#include "freertos/event_groups.h"
#include "secrets.h"

static const char *TAG = "MQTT_RO_WATER_VALVE";

#define WIFI_MAXIMUM_RETRY 5

// MQTT broker information
#define MQTT_BROKER_URL "mqtt://192.168.1.206:1883"

// Device information for Home Assistant auto-discovery
#define DEVICE_NAME "water_valve_controller"
#define DEVICE_MODEL "ESP32CYD"
#define DEVICE_MANUFACTURER "Custom"

// Topic templates for a single valve
#define STATE_TOPIC "water_valve/state"
#define COMMAND_TOPIC "water_valve/set"
#define AVAILABILITY_TOPIC "water_valve/status"
#define DISCOVERY_TOPIC "homeassistant/switch/water_valve/config"

// FreeRTOS event group to signal WiFi connection
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// WiFi connection retry counter
static int s_retry_num = 0;

// MQTT client variables
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// Callback type for state change
typedef void (*mqtt_state_change_callback_t)(uint8_t relay_num, bool state);

// Static variables
static mqtt_state_change_callback_t state_change_callback = NULL;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// Function prototypes
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void publish_discovery_info(void);
static void handle_valve_command(const char* payload, int payload_len);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_init_sta(void);

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi as station
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Wait for WiFi connection */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Unexpected event");
    }
}

bool mqtt_is_connected(void) {
    return mqtt_connected;
}

bool mqtt_init(void) {
    ESP_LOGI(TAG, "Initializing MQTT client");
    
    // Initialize NVS - it is used to store PHY calibration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .broker.verification.skip_cert_common_name_check = true,
        .session.disable_clean_session = 0,
        .network.reconnect_timeout_ms = 10000,
        .network.timeout_ms = 10000,
        // Optional: Add credentials if your broker requires authentication
        .credentials.username = "mqtt",
        .credentials.authentication.password = "mqtt",
    };
    
    // Initialize MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return false;
    }
    
    // Register event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // Start MQTT client
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        // Don't fail the initialization - we'll keep trying to reconnect
    }
    
    return true;
}

static void publish_discovery_info(void) {
    if (!mqtt_is_connected()) {
        ESP_LOGD(TAG, "MQTT not connected, skipping discovery publish");
        return;
    }
    
    char message[512];
    
    // Create discovery message (JSON)
    snprintf(message, sizeof(message), 
        "{"
        "\"name\":\"Water Valve\","
        "\"unique_id\":\"water_valve_controller\","
        "\"state_topic\":\"%s\","
        "\"command_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"%s\","
            "\"manufacturer\":\"%s\""
        "}"
        "}",
        STATE_TOPIC, COMMAND_TOPIC, AVAILABILITY_TOPIC,
        DEVICE_NAME, DEVICE_NAME, DEVICE_MODEL, DEVICE_MANUFACTURER
    );
    
    // Publish discovery message (retained)
    int msg_id = esp_mqtt_client_publish(mqtt_client, DISCOVERY_TOPIC, message, 0, 1, 1);
    if (msg_id != -1) {
        ESP_LOGI(TAG, "Published discovery info for water valve");
    } else {
        ESP_LOGW(TAG, "Failed to publish discovery info for water valve");
    }
}

void mqtt_publish_relay_state(uint8_t relay_num, bool state) {
    if (!mqtt_is_connected()) {
        ESP_LOGD(TAG, "MQTT not connected, skipping valve state publish");
        return;
    }
    
    // Publish state (retained)
    int msg_id = esp_mqtt_client_publish(mqtt_client, STATE_TOPIC, state ? "ON" : "OFF", 0, 1, 1);
    if (msg_id != -1) {
        ESP_LOGI(TAG, "Published water valve state: %s", state ? "ON" : "OFF");
    } else {
        ESP_LOGW(TAG, "Failed to publish water valve state");
    }
}

// This function is kept for compatibility but simply calls mqtt_publish_relay_state
void mqtt_publish_all_relay_states(void) {
    // Since we now only have one valve, just update its state
    // The relay_num parameter is ignored, but we keep it for API compatibility
    mqtt_publish_relay_state(1, false);
}

void mqtt_register_state_change_callback(mqtt_state_change_callback_t callback) {
    state_change_callback = callback;
}

static void handle_valve_command(const char* payload, int payload_len) {
    // Null-terminate the payload for string comparison
    char cmd[16] = {0};
    if (payload_len < sizeof(cmd) - 1) {
        memcpy(cmd, payload, payload_len);
    } else {
        memcpy(cmd, payload, sizeof(cmd) - 1);
    }
    
    // Convert payload to valve state
    bool state = false;
    if (strcmp(cmd, "ON") == 0) {
        state = true;
    } else if (strcmp(cmd, "OFF") == 0) {
        state = false;
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
        return;
    }
    
    // Set valve state and publish confirmation
    ESP_LOGI(TAG, "Setting water valve to %s via MQTT", state ? "ON" : "OFF");
    mqtt_publish_relay_state(1, state);
    
    // Notify UI about state change if callback is registered
    if (state_change_callback != NULL) {
        state_change_callback(1, state);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;
            
            // Publish availability status (retained)
            esp_mqtt_client_publish(mqtt_client, AVAILABILITY_TOPIC, "online", 0, 1, 1);
            
            // Publish discovery information for the water valve
            publish_discovery_info();
            
            // Subscribe to command topic
            esp_mqtt_client_subscribe(mqtt_client, COMMAND_TOPIC, 0);
            ESP_LOGI(TAG, "Subscribed to %s", COMMAND_TOPIC);
            
            // Publish current state (default to OFF at startup)
            mqtt_publish_relay_state(1, false);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
            
            // Check if this is a command message
            if (event->topic_len == strlen(COMMAND_TOPIC) && 
                strncmp(event->topic, COMMAND_TOPIC, event->topic_len) == 0) {
                // This is a command for the water valve
                handle_valve_command(event->data, event->data_len);
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
            
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}