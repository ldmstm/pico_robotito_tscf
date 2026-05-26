#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "lwip/sockets.h"
#include <nvs_flash.h>
#include <string.h>

#include "rosout.h"
#include "picoros.h"
#include "picorosso.h"
#include "picoserdes.h"
#include "ticker.h"
#include "wifi_connection_c.h"

#define WIFI_SSID "$"
#define WIFI_PASSWORD "julian1234"
#define ZENOH_ROUTER_ADDRESS "tcp/10.227.207.75:7447"
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
    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    wifi_wait_connected();
    ESP_LOGI(TAG, "WiFi connected.");

    picorosso_setup(ZENOH_NODE_NAME, ZENOH_ROUTER_ADDRESS, ROS_DOMAIN_ID);

    ticker_setup("tick");

    ESP_LOGI(TAG, "Booting completed.");

    for (;;)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        rosout_out("Alive!", __FILE__, __func__, __LINE__, ROSLOG_INFO);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10000));
    }
}
