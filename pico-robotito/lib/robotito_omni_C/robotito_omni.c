#include "picorosso.h"
#include "robotito_omni.h"

#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "fpid.h"
#include "odom_helper.h"
#include "driver/mcpwm.h"
#include "ESP32Encoder.h"

#define NMOTORS 3
static const uint8_t motor_pins[] = {27, 26, 37, 39, 23, 18, 35, 34, 25, 33, 36, 38}; // v>=6
// static const uint8_t motor_pins[] = {27, 26, 37, 39, 23, 18, 35, 34, 33, 25, 36, 38}; // v<6

// MCPWM peripheral components to use
static const mcpwm_unit_t drv8833_mcpwm_unit = MCPWM_UNIT_0;
static const uint32_t drv8833_mcpwm_freq = 50000;

static const float ROBOT_R = 0.0675; // m
static const float ROBOT_R_3 = 1.0 / (ROBOT_R * 3.0);

static const float WHEEL_DIAMETER = 0.038; // m
static const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

static const float ENC_CPR = 3;
static const float MOTOR_REDUCTION = 50;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float M_PI_3 = M_PI / 3.0f;
static const float M_PI_6 = M_PI / 6.0f;
static const float M_2_3 = 2.0f / 3.0f;

static const float TICS_PER_REVOLUTION = ENC_CPR * MOTOR_REDUCTION;
static const float RAD_PER_TICK = 2.0 * M_PI / TICS_PER_REVOLUTION;

static const float M_SEC_TO_TICS_SEC = 1 / (RAD_PER_TICK * WHEEL_RADIUS);

static const float MAX_SPEED_POWER = 90;  // power % at which MAX_SPEED_TICS is obtained
static const float MAX_SPEED_TICS = 1080; // tics/s at MAX_SPEED_POWER

// static const float MAX_SPEED_RAD = MAX_SPEED_TICS * RAD_PER_TICK; // rad/s
// static const float MAX_SPEED_LIN = MAX_SPEED_RAD * WHEEL_RADIUS;  // m/s

static const float KF = MAX_SPEED_POWER / MAX_SPEED_TICS;
static const float KP = 0.1f; // 0.01; // 0.01;
static const float KI = 0.5f; // 0.05; // 0.05;
static const float KD = 0.0f;

static const int motor_control_period_ms = 20;
static const int mobility_report_period_ms = 200;

#define TSK_MINIMAL_STACK_SIZE (1024)
#define CONTROL_TASK_NAME "omni_control_task"
#define CONTROL_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 9)
#define CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define REPORT_TASK_NAME "omni_report_task"
#define REPORT_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 9)
#define REPORT_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define PUBLISHER_BUF_SIZE 1024 // TODO

static const char *TAG = "omni";

static uint8_t publisher_buf[PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

#include "vector_math.h"

static double state_position[3];
static double state_velocity[3];
static double state_efforts[3];

static const char *joint_names[NMOTORS] = {"wheel1", "wheel2", "wheel3"};
static ros_JointState msg_joint_state = {
    .header = {.frame_id = (char *)"base_link"},
    .name = {.data = (char **)joint_names, .n_elements = NMOTORS},
    .position = {.data = state_position, .n_elements = NMOTORS},
    .velocity = {.data = state_velocity, .n_elements = NMOTORS},
    .effort = {.data = state_efforts, .n_elements = NMOTORS},
};
static picoros_publisher_t publisher_joint = {
    .topic = {
        .name = (char *)"joint_states",
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

static void cmd_vel_cb(uint8_t *rx_data, size_t data_len);
picoros_subscriber_t subscription_cmd_vel = {
    .topic = {
        .name = (char *)"cmd_vel",
        .type = ROSTYPE_NAME(ros_TwistStamped),
        .rihs_hash = ROSTYPE_HASH(ros_TwistStamped),
    },
    .user_callback = cmd_vel_cb,
};

static unsigned long set_control_time_ms = 0;
static unsigned long control_cb_time_ms = 0;

typedef struct
{
  mcpwm_timer_t pwm_timer;

  ESP32Encoder encoder;

  float target_v_t_s; // tics_sec

  Fpid pid;
  float output;

  int64_t counter;
  int64_t last_count;
  // float current_v_t_s;

  int64_t last_odom_count;
} servo_t;

static servo_t motors[NMOTORS];
// static odom_t odometry_position;
// static odom_t odometry_velocities;
int64_t last_odom_time = 0;

static vec3_t getW(float x_dot, float y_dot, float w_dot, float phi_r)
{
  vec3_t v = vec3(x_dot, y_dot, -w_dot);

  mat3_t M = mat3(
      -sin(phi_r), cos(phi_r), ROBOT_R,
      -sin(M_PI_3 - phi_r), -cos(M_PI_3 - phi_r), ROBOT_R,
      sin(M_PI_3 + phi_r), -cos(M_PI_3 + phi_r), ROBOT_R);

  vec3_t w = mat3_mul_vec3(M, v);
  return w;
}

static vec3_t getInverseW(float w_1, float w_2, float w_3, float phi)
{
  vec3_t x = vec3(w_1 * WHEEL_RADIUS, w_2 * WHEEL_RADIUS, w_3 * WHEEL_RADIUS);

  mat3_t A = mat3(
      -sin(phi) * M_2_3, -cos(M_PI_6 + phi) * M_2_3, cos(phi - M_PI_6) * M_2_3,
      cos(phi) * M_2_3, -sin(M_PI_6 + phi) * M_2_3, sin(phi - M_PI_6) * M_2_3,
      ROBOT_R_3, ROBOT_R_3, ROBOT_R_3);

  vec3_t u = mat3_mul_vec3(A, x);

  return u;
}

static void compute_movement(float time_step)
{
  if (time_step == 0.0f)
  {
    // ESP_LOGW(TAG, "compute_movement called with zero time_step, skipping.");
    return;
  }
  int64_t count;
  for (int i = 0; i < NMOTORS; i++)
  {
    count = ESP32Encoder_getCount(&motors[i].encoder);
    motors[i].last_count = count - motors[i].counter;
    motors[i].counter = count;
  }

  // reverse kinematics
  int64_t odom_increment0 = motors[0].counter - motors[0].last_odom_count;
  int64_t odom_increment1 = motors[1].counter - motors[1].last_odom_count;
  int64_t odom_increment2 = motors[2].counter - motors[2].last_odom_count;

  vec3_t odom_vels = getInverseW(
      odom_increment0 * RAD_PER_TICK / time_step,
      odom_increment1 * RAD_PER_TICK / time_step,
      odom_increment2 * RAD_PER_TICK / time_step,
      0); //-odom.pos.phi);
  
  OdomHelper_update_pos(odom_vels.value[0], odom_vels.value[1], -odom_vels.value[2], time_step);

  motors[0].last_odom_count = motors[0].counter;
  motors[1].last_odom_count = motors[1].counter;
  motors[2].last_odom_count = motors[2].counter;
}

static void set_channel(bool bBrake, float power, mcpwm_timer_t timer)
{
  if (bBrake)
  {
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 100);
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 100);
  }
  else if (power > 0)
  {
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, fabsf(power));
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 0);
  }
  else if (power < 0)
  {
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 0);
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, fabsf(power));
  }
  else
  {
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 0);
    mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 0);
  }
}

static void report_task(void *pvParameters)
{
  float step = mobility_report_period_ms / 1000.0f;
  for (;;)
  {
    TickType_t last_wake_time = xTaskGetTickCount();
    picorosso_set_timestamp(&msg_joint_state.header.stamp);

    for (int i = 0; i < NMOTORS; i++)
    {
      double new_position = motors[i].last_odom_count * RAD_PER_TICK;
      double angle_moved = new_position - state_position[i];
      state_position[i] = new_position;
      state_velocity[i] = angle_moved / step;
    }

    pr_publish_buf(publisher_joint, msg_joint_state, publisher_buf, sizeof(publisher_buf));

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(mobility_report_period_ms));
  }
}

static void control_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;)
  {
    uint64_t now_ms = pdTICKS_TO_MS(last_wake_time);
    uint64_t time_step_ms = now_ms - control_cb_time_ms;

    if (time_step_ms > 0)
    {
      control_cb_time_ms = now_ms;
      float time_step = (float)time_step_ms / 1000.0f;

      compute_movement(time_step);

      if (((now_ms - set_control_time_ms) > STOP_TIMEOUT_MS))
      {
        ESP_LOGD(TAG, "control input timeout.");
        set_control_time_ms = now_ms;
        RobotitoOmni_stop();
      }
      else
      {
        for (int i = 0; i < NMOTORS; i++)
        {
          float current_v_t_s = motors[i].last_count / time_step;
          float output = Fpid_compute(&motors[i].pid, motors[i].target_v_t_s, current_v_t_s, time_step);

          set_channel(false, output, motors[i].pwm_timer);
          motors[i].output = output;
        }
      }
    }
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(motor_control_period_ms));
  }
}

static void cmd_vel_cb(uint8_t *rx_data, size_t data_len)
{
  set_control_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());

  ros_TwistStamped msg_cmd_vel = {0};
  if (ps_deserialize(rx_data, &msg_cmd_vel, data_len))
  {
    // TODO verify msg_cmd_vel.header.stamp

    float x_dot = msg_cmd_vel.twist.linear.x;
    float y_dot = msg_cmd_vel.twist.linear.y;
    float w_dot = msg_cmd_vel.twist.angular.z;
    float phi = 0.0; // TODO luaL_optnumber( L, 4, 0.0 );

    vec3_t w = getW(x_dot, y_dot, w_dot, phi);
    for (int i = 0; i < NMOTORS; i++)
    {
      motors[i].target_v_t_s = w.value[i] * M_SEC_TO_TICS_SEC;
    }

    if (x_dot == 0 && y_dot == 0 && w_dot == 0)
    {
      for (int i = 0; i < NMOTORS; i++)
      {
        Fpid_reset_errors(&motors[i].pid, 0.0f);
      }
    }
  }
  else
  {
    ESP_LOGE(TAG, "cmd_vel message deserialization error");
  }
}

// static void disable_motors()
// {
//   for (int i = 0; i < NMOTORS; i++)
//   {
//     set_channel(false, 0.0f, motors[i].pwm_timer);
//   }
// }

static void enable_motors()
{
  for (int i = 0; i < NMOTORS; i++)
  {
    Fpid_reset_errors(&motors[i].pid, 0.0f);
    ESP32Encoder_clearCount(&motors[i].encoder);
    motors[i].target_v_t_s = 0.0;
    motors[i].counter = 0;
    motors[i].last_count = 0;
    motors[i].last_odom_count = 0;
  }
  set_control_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
}

static bool init_motors()
{
  ESP_LOGD(TAG, "Configuring motors...");

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motor_pins[4 * 0]);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motor_pins[4 * 0 + 1]);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, motor_pins[4 * 1]);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, motor_pins[4 * 1 + 1]);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, motor_pins[4 * 2]);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, motor_pins[4 * 2 + 1]);

  // 2. initial mcpwm configuration
  mcpwm_config_t pwm_config;
  pwm_config.frequency = drv8833_mcpwm_freq; // 1000;    //frequency = 1000Hz,
  pwm_config.cmpr_a = 0;                     // duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;                     // duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings
  mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_1, &pwm_config); // Configure PWM1A & PWM1B with above settings
  mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_2, &pwm_config); // Configure PWM1A & PWM1B with above settings

  motors[0].pwm_timer = MCPWM_TIMER_0;
  motors[1].pwm_timer = MCPWM_TIMER_1;
  motors[2].pwm_timer = MCPWM_TIMER_2;

  // Enable the weak pull up resistors
  ESP32Encoder_useInternalWeakPullResistors(up);

  for (int i = 0; i < NMOTORS; i++)
  {
    // int8_t pin1 = motor_pins[4 * i];
    // int8_t pin2 = motor_pins[4 * i + 1];
    int8_t encA = motor_pins[4 * i + 2];
    int8_t encB = motor_pins[4 * i + 3];

    ESP_LOGI(TAG, "Setting motor %d pins:%d,%d enc:%d,%d... ", i, motor_pins[4 * i], motor_pins[4 * i + 1], encA, encB);

    // encoder
    ESP32Encoder_init(&motors[i].encoder, false, NULL, NULL);
    ESP32Encoder_attachSingleEdge(&motors[i].encoder, encA, encB);
    ESP32Encoder_clearCount(&motors[i].encoder);

    // pid
    Fpid_init_with_limits(&motors[i].pid, -MAX_SPEED_POWER, MAX_SPEED_POWER);
    Fpid_update_k_independent(&motors[i].pid, KF, KP, KI, KD);
    motors[i].target_v_t_s = 0.0;
  }
  ESP_LOGD(TAG, "Motors configured.");

  OdomHelper_reset();

  return true;
}

void RobotitoOmni_stop()
{
  for (int i = 0; i < NMOTORS; i++)
  {
    motors[i].target_v_t_s = 0.0;
    Fpid_reset_errors(&motors[i].pid, 0.0f);
    set_channel(false, 0.0f, motors[i].pwm_timer);
  }
}

bool RobotitoOmni_setup(const char *topic_cmd_vel,
                        const char *topic_odom)
{
  ESP_LOGD(TAG, "setup mobility: robotito... ");

  if (!init_motors())
  {
    ESP_LOGE(TAG, "failure initializing motors.");
    return false;
  }
  enable_motors();

  if (!OdomHelper_setup(topic_odom, "tf", "odom", "base_footprint"))
  {
    ESP_LOGE(TAG, "failure initializing odom_helper.");
    return false;
  }

  ESP_LOGD(TAG, "setup: control...");

  ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_joint.topic.name);
  picoros_publisher_declare(&picorosso_node, &publisher_joint);

  subscription_cmd_vel.topic.name = (char *)topic_cmd_vel;
  ESP_LOGI(TAG, "Declaring subscriber on [%s]", subscription_cmd_vel.topic.name);
  picoros_subscriber_declare(&picorosso_node, &subscription_cmd_vel);

  control_cb_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
  
  xTaskCreate(
      control_task,
      CONTROL_TASK_NAME,
      CONTROL_TASK_STACK_SIZE,
      NULL,
      CONTROL_TASK_PRIORITY,
      NULL);

  xTaskCreate(
      report_task,
      REPORT_TASK_NAME,
      REPORT_TASK_STACK_SIZE,
      NULL,
      REPORT_TASK_PRIORITY,
      NULL);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
