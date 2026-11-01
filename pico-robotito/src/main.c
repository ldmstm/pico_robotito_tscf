/*
 * Driver includes
 */
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <string.h>

/*
 * External lib includes
 */
#include "picoros.h"
#include "picorosso.h"
#include "rosout.h"
#include "picoserdes.h"
#include "ticker.h"
#include "wifi_connection.h"

/*
 * Internal lib includes
 */
#include "robotito_omni.h"
#include "robotito_vl53ring.h"
#include <driver/i2c_master.h>

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWD"
#define ZENOH_ROUTER_ADDRESS "tcp/xxx.xxx.xxx.xxx:7447"
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

    ESP_LOGI(TAG, "Setting up RobotitoOmni...");
    if (!RobotitoOmni_setup("cmd_vel", "odom"))
    {
        ESP_LOGE(TAG, "FAIL RobotitoOmni_setup()");
    }
    ESP_LOGI(TAG, "RobotitoOmni setup complete.");

    ESP_LOGI(TAG, "Initializing I2C master bus...");
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

    ESP_LOGI(TAG, "Setting up RobotitoVl53Ring...");
    if (!RobotitoVl53Ring_setup(i2c_bus_handle))
    {
        ESP_LOGE(TAG, "FAIL RobotitoVl53Ring_setup()");
    }
    ESP_LOGI(TAG, "RobotitoVl53Ring setup complete.");

    ESP_LOGI(TAG, "Booting completed.");

    for (;;)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        ESP_LOGD(TAG, "Sending Alive! heartbeat");
        rosout_out("Alive!", __FILE__, __func__, __LINE__, ROSLOG_INFO); //TODO: level should be first parameter
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(30000));
    }
}
