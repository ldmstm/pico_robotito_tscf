#ifndef __ROBOTITO_OMNI_H
#define __ROBOTITO_OMNI_H

#include <stdbool.h>

#define STOP_TIMEOUT_MS (4 * 1000)

bool RobotitoOmni_setup(const char *topic_cmd_vel,
                        const char *topic_odom);

void RobotitoOmni_stop();

#endif // __ROBOTITO_OMNI_H
