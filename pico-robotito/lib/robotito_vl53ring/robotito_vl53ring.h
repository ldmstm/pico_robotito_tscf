#ifndef __ROBOTITO_VL53RING_H
#define __ROBOTITO_VL53RING_H

#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

bool RobotitoVl53Ring_setup(i2c_master_bus_handle_t bus_handle,
                            const char *topic_scan);

#ifdef __cplusplus
}
#endif

#endif // __ROBOTITO_VL53RING_H
