#include "picorosso.h"
#include "robotito_vl53ring.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
//extern "C"
//{
#include "vl53l0x.h"
//}
#include <math.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "vl53ring";

#define I2C_PORT I2C_NUM_0
#define I2C_SCL 22
#define I2C_SDA 21

#define N_SENSORS 6
#define SENSOR_RADIUS 0.07f // distance to sensor from roboti's center
#define ADDRESS_DEFAULT 0b0101001
#define CONINUOUS_PERIOD_MS (uint16_t)100 // sampling period (ie timer period) / 2

typedef struct
{
  gpio_num_t xshut_pin;
  uint8_t address;
  vl53l0x_t *vl53l0x;
} sensor_t;

// TODO verify: "angles are measured around the positive Z axis (counterclockwise, if Z is up)
//  with zero angle being forward along the x axis"
static sensor_t sensor_defs[N_SENSORS] = {
    {.xshut_pin = GPIO_NUM_16, .address = ADDRESS_DEFAULT + 1},
    {.xshut_pin = GPIO_NUM_17, .address = ADDRESS_DEFAULT + 2},
    {.xshut_pin = GPIO_NUM_2, .address = ADDRESS_DEFAULT + 3},
    {.xshut_pin = GPIO_NUM_14, .address = ADDRESS_DEFAULT + 4},
    {.xshut_pin = GPIO_NUM_12, .address = ADDRESS_DEFAULT + 5},
    {.xshut_pin = GPIO_NUM_13, .address = ADDRESS_DEFAULT + 6},
};

#define TSK_MINIMAL_STACK_SIZE (1024)
#define I2C0_TASK_NAME "vl53ring_task"
#define I2C0_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define I2C0_TASK_PRIORITY (tskIDLE_PRIORITY + 0)
#define PUBLISHER_BUF_SIZE 1024 // TODO

static uint8_t publisher_buf[PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

static picoros_publisher_t publisher_laser_scan = {
    .topic =
        {
            .name = NULL,
            .type = ROSTYPE_NAME(ros_LaserScan),
            .rihs_hash = ROSTYPE_HASH(ros_LaserScan),
        },
};

static float laser_ranges[N_SENSORS];
static float laser_intensities[N_SENSORS];
static ros_LaserScan msg_laser_scan = {
    .header = {.frame_id = (char *)"laser_link"},
    .ranges = {
        .data = laser_ranges,
        .n_elements = N_SENSORS,
    },
    .intensities = {
        .data = laser_intensities,
        .n_elements = N_SENSORS,
    }};

Vl53Ring::Vl53Ring()
{
  msg_laser_scan.angle_min = -M_PI / 2;                           // 0.0;
  msg_laser_scan.angle_max = msg_laser_scan.angle_min + 2 * M_PI; // 2 * M_PI;
  msg_laser_scan.angle_increment = 2 * M_PI / N_SENSORS;
  msg_laser_scan.time_increment = 0.0; // FIXME
  msg_laser_scan.scan_time = CONINUOUS_PERIOD_MS / 1000.0;
  msg_laser_scan.range_min = SENSOR_RADIUS + 0.003;
  msg_laser_scan.range_max = SENSOR_RADIUS + 1.2; // 1.2 in default, 2 in long range
}

bool Vl53Ring::initialize_ring(i2c_master_bus_handle_t &bus_handle)
{
  // put all sensors in reset
  for (int i = 0; i < N_SENSORS; i++)
  {
    gpio_num_t pin = sensor_defs[i].xshut_pin;
    gpio_config_t io_conf_wait = {
        .pin_bit_mask = (1ULL << pin),         // Select GPIO
        .mode = GPIO_MODE_OUTPUT,              // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };
    gpio_config(&io_conf_wait);
    gpio_set_level(pin, 0);
  }

  z_sleep_ms(10);

  // init sensors and renumber
  for (int i = 0; i < N_SENSORS; i++)
  {
    gpio_num_t pin = sensor_defs[i].xshut_pin;
    uint8_t address = sensor_defs[i].address;
    gpio_set_level(pin, 1);
    z_sleep_ms(10);

    sensor_defs[i].vl53l0x = vl53l0x_config(
        bus_handle, pin, address, true);

    if (sensor_defs[i].vl53l0x == NULL)
    {
      ESP_LOGE(TAG, "Failure configuring vl53 %d on pin %d", i, pin);
    }

    const char *err;
    err = vl53l0x_init(sensor_defs[i].vl53l0x);
    if (strcmp(err, "OK") != 0)
    {
      ESP_LOGE(TAG, "Faile initializing vl53 %d on pin %d", i, pin);
    }

    vl53l0x_setAddress(sensor_defs[i].vl53l0x, address);
    z_sleep_ms(10);
  }

  for (int i = 0; i < N_SENSORS; i++)
  {
    vl53l0x_startContinuous(sensor_defs[i].vl53l0x, CONINUOUS_PERIOD_MS);
  }

  return true;
}

/*
static void control_cb() {
}
*/

static void i2c0_vl53ring_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {

    bool new_reading = false;
    for (int i = 0; i < N_SENSORS; i++)
    {
      uint16_t range = vl53l0x_readRangeContinuousMillimeters(sensor_defs[i].vl53l0x);
      if (range < 65535)
      {
        new_reading = true;
        laser_ranges[i] = SENSOR_RADIUS + range / 1000.0; // mm -> m
        /*
        if (i==0 || i==N_SENSORS-1) {
          char cmd_str[20] {0};
          sprintf(cmd_str, "> %d %f", i, ranges[i]);
          micro_rosso::logger.log(cmd_str);
        }
        */
      }
    }
    if (new_reading)
    {
      PicoRosso::set_timestamp(msg_laser_scan.header.stamp);
      pr_publish_buf(publisher_laser_scan, msg_laser_scan, publisher_buf, sizeof(publisher_buf));
    }

    // pause the task per defined wait period
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONINUOUS_PERIOD_MS));
  }
}

bool Vl53Ring::setup(i2c_master_bus_handle_t &bus_handle,
                     const char *topic_scan)
{
  ESP_LOGD(TAG, "Setting up...");

  initialize_ring(bus_handle);

  ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_scan);
  publisher_laser_scan.topic.name = topic_scan;
  picoros_publisher_declare(&PicoRosso::node, &publisher_laser_scan);

  /* create task pinned to the app core */
  xTaskCreatePinnedToCore(
      i2c0_vl53ring_task,
      I2C0_TASK_NAME,
      I2C0_TASK_STACK_SIZE,
      NULL,
      I2C0_TASK_PRIORITY,
      NULL,
      APP_CPU_NUM);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
