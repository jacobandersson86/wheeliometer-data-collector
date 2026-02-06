#include "wifi.hpp"
#include "terminal.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <string.h>

static const char* TAG = "WIFI";
static bool wifi_initialized = false;
static bool ap_running = false;
static int connected_stations = 0;

// Event handler for WiFi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
                connected_stations++;
                ESP_LOGI(TAG, "Station " MACSTR " joined, total: %d",
                        MAC2STR(event->mac), connected_stations);
                terminal_write("WiFi: Station connected (%d total)", connected_stations);
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
                connected_stations--;
                ESP_LOGI(TAG, "Station " MACSTR " left, total: %d",
                        MAC2STR(event->mac), connected_stations);
                terminal_write("WiFi: Station disconnected (%d total)", connected_stations);
                break;
            }
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "Access Point started");
                terminal_write("WiFi AP started: Wheeliometer");
                break;
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "Access Point stopped");
                terminal_write("WiFi AP stopped");
                break;
            default:
                break;
        }
    }
}

bool wifi_init() {
    if (wifi_initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return true;
    }

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi AP netif
    esp_netif_create_default_wifi_ap();

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

    // Set WiFi mode to AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi initialized");
    terminal_write("WiFi initialized");

    return true;
}

bool wifi_start_ap() {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return false;
    }

    if (ap_running) {
        ESP_LOGW(TAG, "AP already running");
        return true;
    }

    // Configure AP
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, "Wheeliometer");
    wifi_config.ap.ssid_len = strlen("Wheeliometer");
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;  // No password

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    ap_running = true;
    connected_stations = 0;

    ESP_LOGI(TAG, "Access Point started: SSID=Wheeliometer");

    return true;
}

bool wifi_stop_ap() {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return false;
    }

    if (!ap_running) {
        ESP_LOGW(TAG, "AP not running");
        return true;
    }

    // Stop WiFi
    ESP_ERROR_CHECK(esp_wifi_stop());

    ap_running = false;
    connected_stations = 0;

    ESP_LOGI(TAG, "Access Point stopped");

    return true;
}

bool wifi_is_ap_running() {
    return ap_running;
}

int wifi_get_station_count() {
    return connected_stations;
}
