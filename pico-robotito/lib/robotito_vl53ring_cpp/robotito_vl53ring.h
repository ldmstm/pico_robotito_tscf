#ifndef __robotito_vl53ring
#define __robotito_vl53ring

#include "driver/i2c_master.h"

class Vl53Ring
{
public:
  Vl53Ring();
  static bool setup(i2c_master_bus_handle_t &bus_handle,
                    const char *topic_scan = "scan");

private:
  static bool initialize_ring(i2c_master_bus_handle_t &bus_handle);
};

#endif // __robotito_vl53ring
