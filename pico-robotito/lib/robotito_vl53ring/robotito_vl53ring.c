#include "picorosso.h"
#include "robotito_vl53ring.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "vl53l0x.h"
#include <math.h>
#include <stdio.h> // snprintf for the per-sensor topic and frame names
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "vl53ring";

#define I2C_PORT I2C_NUM_0
#define I2C_SCL 22
#define I2C_SDA 21

#define N_SENSORS 6
#define SENSOR_RADIUS 0.07f // distance to sensor from robotito's center
#define ADDRESS_DEFAULT 0b0101001
#define CONINUOUS_PERIOD_MS 100 // sampling period / 2

#define SENSOR_ARC_RAD (2.0f * M_PI / N_SENSORS)
#define RANGE_MIN 0.0f
#define RANGE_MAX 2.0f
#define RANGE_VARIANCE 0.01f
#define VL53_NO_READING 65535
#define RANGE_TOPIC_NAME_LEN  32

#define TSK_MINIMAL_STACK_SIZE (1024)
#define I2C0_TASK_NAME "vl53ring_task"
#define I2C0_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define I2C0_TASK_PRIORITY (tskIDLE_PRIORITY + 0)

typedef struct
{
  gpio_num_t xshut_pin;
  uint8_t address;
  vl53l0x_t *vl53l0x;
  float heading_rad;
  const char *name;

  ros_Range msg;
  picoros_publisher_t pub;
  uint8_t pub_buf[256];
} sensor_t;

#define DECLARE_MSG_RANGE(_fid, _radtype, _fov, _minr, _maxr, _range, _var) \
  (ros_Range){                                                              \
    .header = { .frame_id = (_fid) },                                       \
    .radiation_type = (_radtype),                                           \
    .field_of_view = (_fov),                                                \
    .min_range = (_minr),                                                   \
    .max_range = (_maxr),                                                   \
    .range = (_range),                                                      \
    .variance = (_var),                                                     \
  }

#define DECLARE_PUB_RANGE(_id)              \
  (picoros_publisher_t){                    \
    .topic = {                              \
      .name = "sensor_" #_id "_range",     \
      .type = ROSTYPE_NAME(ros_Range),      \
      .rihs_hash = ROSTYPE_HASH(ros_Range), \
    }                                       \
  }

#define DECLARE_I2C_VL53(_id, _xs_pin, _addr, _hdng)  \
  (sensor_t){                                         \
    .xshut_pin = (_xs_pin),                           \
    .address = (_addr),                               \
    .vl53l0x = NULL, /* init at runtime */            \
    .heading_rad = (_hdng),                           \
                                                      \
    .msg = DECLARE_MSG_RANGE(                         \
      "sensor_" #_id "_link",                         \
      1, SENSOR_ARC_RAD, RANGE_MIN, RANGE_MAX,        \
      RANGE_MAX, RANGE_VARIANCE                       \
    ),                                                \
                                                      \
    .pub = DECLARE_PUB_RANGE(_id),                    \
    .pub_buf = {},                                    \
  }

/*
 * Sensor 0 is mounted at the front (+X of base_link); the rest follow clockwise
 * as installed, so headings decrease by 60 deg. TODO(verify) the physical mount
 * order against the chassis before trusting the map: a wrong order rotates/mirrors
 * the whole ring, which is invisible in the log but obvious in rviz.
 */
static sensor_t sensor_defs[N_SENSORS] = {
  DECLARE_I2C_VL53(0, GPIO_NUM_16, ADDRESS_DEFAULT + 1, 0.0f),
  DECLARE_I2C_VL53(1, GPIO_NUM_17, ADDRESS_DEFAULT + 2, -M_PI / 3.0f),
  DECLARE_I2C_VL53(2, GPIO_NUM_2,  ADDRESS_DEFAULT + 3, (-2 * M_PI) / 3.0f),
  DECLARE_I2C_VL53(3, GPIO_NUM_14, ADDRESS_DEFAULT + 4, M_PI),
  DECLARE_I2C_VL53(4, GPIO_NUM_12, ADDRESS_DEFAULT + 5, (2.0 * M_PI) / 3.0f),
  DECLARE_I2C_VL53(5, GPIO_NUM_13, ADDRESS_DEFAULT + 6, M_PI / 3.0f),
};

static bool initialize_ring(i2c_master_bus_handle_t bus_handle)
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

    sensor_defs[i].vl53l0x = vl53l0x_config(bus_handle, pin, true);

    if (sensor_defs[i].vl53l0x == NULL)
    {
      ESP_LOGE(TAG, "Failure configuring vl53 %d on pin %d", i, pin);
    }

    const char *err;
    err = vl53l0x_init(sensor_defs[i].vl53l0x);
    if (err != NULL)
    {
      ESP_LOGE(TAG, "Failed initializing vl53 %d on pin %d: %s", i, pin, err);
    }

    vl53l0x_setAddress(sensor_defs[i].vl53l0x, address);
    z_sleep_ms(10);
  }

  for (int i = 0; i < N_SENSORS; i++)
  {
    vl53l0x_startContinuous(sensor_defs[i].vl53l0x, CONINUOUS_PERIOD_MS);
  }

  for (int i = 0; i < N_SENSORS; i++)
  {
    ESP_LOGI(TAG, "Declaring publisher on [%s]", sensor_defs[i].pub.topic.name);
    // publisher_laser_scan.topic.name = (char *)topic_scan;
    picoros_publisher_declare(&picorosso_node, &sensor_defs[i].pub);
  }

  return true;
}

static void i2c0_vl53ring_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    for (int i = 0; i < N_SENSORS; i++)
    {
      sensor_t *s = &sensor_defs[i];
      if (s->vl53l0x == NULL)
        continue;

      uint16_t range_mm = vl53l0x_readRangeContinuousMillimeters(s->vl53l0x);
      ESP_LOGW(TAG, "range[%d] = %d", i, range_mm);

      /*
       * REP 117: +Inf means "no target within range". RangeSensorLayer treats a
       * reading >= max_range as a clear when clear_on_max_reading is true, which
       * empties the cone instead of leaving a stale mark. NaN readings are
       * silently dropped by the layer, so +Inf is the value we want here.
       */
      s->msg.range = (range_mm >= VL53_NO_READING) ? INFINITY : range_mm / 1000.0f;
      picorosso_set_timestamp(&s->msg.header.stamp);
      pr_publish_buf(s->pub, s->msg, s->pub_buf, sizeof(s->pub_buf));
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONINUOUS_PERIOD_MS));
  }
}

bool RobotitoVl53Ring_setup(i2c_master_bus_handle_t bus_handle)
{
  ESP_LOGD(TAG, "Setting up...");

  initialize_ring(bus_handle);

  xTaskCreate(
      i2c0_vl53ring_task,
      I2C0_TASK_NAME,
      I2C0_TASK_STACK_SIZE,
      NULL,
      I2C0_TASK_PRIORITY,
      NULL);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
