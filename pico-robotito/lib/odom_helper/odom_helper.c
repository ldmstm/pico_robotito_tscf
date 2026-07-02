#include <esp_log.h>
#include "picorosso.h"
#include "odom_helper.h"
#include <math.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TSK_MINIMAL_STACK_SIZE (1024)
#define ODOM_TASK_NAME "odom_helper_task"
#define ODOM_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define ODOM_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define PUBLISHER_BUF_SIZE 1024 // TODO

static const char *TAG = "odom";

static uint8_t publisher_buf[PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

odom_t OdomHelper_pos = {0.0, 0.0, 0.0};
odom_t OdomHelper_vel = {0.0, 0.0, 0.0};

// Odometry message and publisher
static ros_Odometry msg_odom;
picoros_publisher_t publisher_odom = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Odometry),
        .rihs_hash = ROSTYPE_HASH(ros_Odometry),
    },
};

static ros_TransformStamped msg_transform;
static ros_TFMessage msg_tf = {
    .transforms = {
        .data = &msg_transform,
        .n_elements = 1,
    },
};
picoros_publisher_t publisher_tf = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_TFMessage),
        .rihs_hash = ROSTYPE_HASH(ros_TFMessage),
    },
};

static void euler_to_quat(float roll, float pitch, float yaw, float *q)
{
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - sy * sp * sr;
}

void OdomHelper_init()
{
  // we don't change these anymore
  msg_odom.pose.covariance[0] = 0.0001;
  msg_odom.pose.covariance[7] = 0.0001;
  msg_odom.pose.covariance[35] = 0.0001;

  msg_odom.twist.covariance[0] = 0.0001;
  msg_odom.twist.covariance[7] = 0.0001;
  msg_odom.twist.covariance[35] = 0.0001;
}

void OdomHelper_set(float x, float y, float phi)
{
  OdomHelper_pos.x = x;
  OdomHelper_pos.y = y;
  OdomHelper_pos.phi = phi;
}

void OdomHelper_reset()
{
  OdomHelper_set(0.0, 0.0, 0.0);
}

void OdomHelper_update_pos(float vx, float vy, float vphi, float dt)
{
  OdomHelper_vel.x = vx;
  OdomHelper_vel.y = vy;
  OdomHelper_vel.phi = vphi;

  OdomHelper_pos.phi += vphi * dt;
  float phi_y = OdomHelper_pos.phi + M_PI_2; // pi/2

  float dx = vx * dt;
  float dy = vy * dt;

  OdomHelper_pos.x += cosf(OdomHelper_pos.phi) * dx;
  OdomHelper_pos.y += sinf(OdomHelper_pos.phi) * dx;
  OdomHelper_pos.x += cosf(phi_y) * dy;
  OdomHelper_pos.y += sinf(phi_y) * dy;
}

static void odom_helper_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    z_clock_t now;
    clock_gettime(CLOCK_REALTIME, &now);

    // robot's position in x,y,phi
    msg_odom.pose.pose.position.x = OdomHelper_pos.x;
    msg_odom.pose.pose.position.y = OdomHelper_pos.y;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, OdomHelper_pos.phi, q);
    // robot's heading in quaternion
    msg_odom.pose.pose.orientation.x = q[1];
    msg_odom.pose.pose.orientation.y = q[2];
    msg_odom.pose.pose.orientation.z = q[3];
    msg_odom.pose.pose.orientation.w = q[0];

    // speed from encoders
    msg_odom.twist.twist.linear.x = OdomHelper_vel.x;
    msg_odom.twist.twist.linear.y = OdomHelper_vel.y;
    msg_odom.twist.twist.angular.z = OdomHelper_vel.phi;

    picorosso_set_timestamp_now(&msg_odom.header.stamp, &now);

    pr_publish_buf(publisher_odom, msg_odom, publisher_buf, sizeof(publisher_buf));

    msg_transform.transform.translation.x = msg_odom.pose.pose.position.x;
    msg_transform.transform.translation.y = msg_odom.pose.pose.position.y;
    msg_transform.transform.translation.z = msg_odom.pose.pose.position.z;
    msg_transform.transform.rotation = msg_odom.pose.pose.orientation;

    picorosso_set_timestamp_now(&msg_transform.header.stamp, &now);

    // RCNOCHECK(rcl_publish(&pdescriptor_tf.publisher, &msg_tf, NULL));
    pr_publish_buf(publisher_tf, msg_tf, publisher_buf, sizeof(publisher_buf));

    // pause the task per defined wait period
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ODOM_INTERVAL_MS));
  }
}

bool OdomHelper_setup(const char *topic_odom,
                      const char *topic_tf,
                      const char *frame_id,
                      const char *child_frame_id)
{
  ESP_LOGD(TAG, "Setting up...");

  OdomHelper_init();

  ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_odom);
  publisher_odom.topic.name = (char *)topic_odom;
  msg_odom.header.frame_id = (char *)frame_id;
  msg_odom.child_frame_id = (char *)child_frame_id;
  picoros_publisher_declare(&picorosso_node, &publisher_odom);

  ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_tf);
  publisher_tf.topic.name = (char *)topic_tf;
  msg_transform.header.frame_id = (char *)frame_id;
  msg_transform.child_frame_id = (char *)child_frame_id;
  picoros_publisher_declare(&picorosso_node, &publisher_tf);

  xTaskCreate(
      odom_helper_task,
      ODOM_TASK_NAME,
      ODOM_TASK_STACK_SIZE,
      NULL,
      ODOM_TASK_PRIORITY,
      NULL);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
