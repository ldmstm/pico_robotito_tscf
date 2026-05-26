#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>

// fix raro?
#include "lwip/sockets.h"

static const char *TAG = "main";

#define ZENOH_ROUTER_ADDRESS "tcp/192.168.1.4:7447"
//#define ZENOH_ROUTER_ADDRESS "serial/UART_0#baudrate=921600"


#define ZENOH_NODE_NAME "pico_test"
#define ROS_DOMAIN_ID 123

#define WIFI_SSID "OLIVERA2.4GHz"
#define WIFI_PASSWORD "31942649"

#include "picorosso.h"
PicoRosso picorosso;

#if defined(WIFI_SSID)
#include "wifi_connection.h"
    void InitNVS()
    {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
    }
#endif

extern "C"
{
    void app_main(void);
}

void app_main()
{
    ESP_LOGI(TAG, "Booting...");

// Start NVS and Wifi
#if defined(WIFI_SSID)
    InitNVS();
    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID '%s'...", WIFI_SSID);
    WifiConnection::connect(WIFI_SSID, WIFI_PASSWORD);

    uint32_t wait_cycles = 0;
    while (WifiConnection::connected == false)
    {
        if ((wait_cycles % 40) == 0)
        {
            ESP_LOGW(TAG, "Waiting for Wi-Fi connection...");
        }
        ++wait_cycles;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Wi-Fi connected.");
#endif

    // PicoRosso initalization //////////////////////////////
    ESP_LOGI(TAG, "Starting PicoRosso...");
    picorosso.setup(ZENOH_NODE_NAME, ZENOH_ROUTER_ADDRESS, ROS_DOMAIN_ID);
    ESP_LOGI(TAG, "PicoRosso initialized.");

    while (true)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        picorosso.rosout.out("Alive!", __FILE__, __func__, __LINE__, ROSLOG_INFO);
        ESP_LOGI(TAG, "tick!");
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}
