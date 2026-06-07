#ifndef __robotito_omni_h
#define __robotito_omni_h

#define STOP_TIMEOUT_MS (4 * 1000)

class RobotitoOmni
{
public:
  RobotitoOmni();

  static bool setup(const char *topic_cmd_vel = "cmd_vel",
                    const char *topic_odom = "odom");

  static void stop();
};

#endif // __robotito_omni_h
