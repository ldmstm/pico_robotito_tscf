#ifndef __ODOM_HELPER_H
#define __ODOM_HELPER_H

#include <stdbool.h>

#define ODOM_INTERVAL_MS 100

typedef struct
{
  float x;
  float y;
  float phi;
} odom_t;

extern odom_t OdomHelper_pos;
extern odom_t OdomHelper_vel;

bool OdomHelper_setup(const char *topic_odom,
                      const char *topic_tf,
                      const char *frame_id,
                      const char *child_frame_id);

void OdomHelper_update_pos(float vx, float vy, float vphi, float dt);
void OdomHelper_set(float x, float y, float phi);
void OdomHelper_reset();

#endif // __ODOM_HELPER_H
