#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <string.h>

#include "picoros.h"
#include "picorosso.h"
#include "rosout.h"
#include "picoserdes.h"
#include "ticker.h"
#include "wifi_connection.h"

#define WIFI_SSID "VIRSANMAR"
#define WIFI_PASSWORD "27174858" 
#define ZENOH_ROUTER_ADDRESS "tcp/172.21.112.1:7447"
#define ZENOH_NODE_NAME "pico_robotito"
#define ROS_DOMAIN_ID 123

static const char *TAG = "main";

static void
init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase ());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void
app_main (void)
{
    ESP_LOGI(TAG, "Starting booting of Robotito");

    init_nvs();
    ESP_LOGI(TAG, "NVS initialized.");

    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    ESP_LOGI(TAG, "WiFi connection started for SSID: %s", WIFI_SSID);

    wifi_wait_connected();
    ESP_LOGI(TAG, "WiFi connected.");

    ESP_LOGI(TAG, "Initializing PicoRosso...");
    if (picorosso_setup(ZENOH_NODE_NAME, ZENOH_ROUTER_ADDRESS, ROS_DOMAIN_ID)) {
        ESP_LOGI(TAG, "PicoRosso initialized successfully.");
    } else {
        ESP_LOGE(TAG, "PicoRosso initialization FAILED.");
    }

    ESP_LOGI(TAG, "Setting up ticker...");
    ticker_setup("tick");
    ESP_LOGI(TAG, "Ticker setup complete.");

    ESP_LOGI(TAG, "Booting completed.");

    for (;;)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        ESP_LOGD(TAG, "Sending Alive! heartbeat");
        rosout_out("Alive!", __FILE__, __func__, __LINE__, ROSLOG_INFO);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(30000));
    }
}
