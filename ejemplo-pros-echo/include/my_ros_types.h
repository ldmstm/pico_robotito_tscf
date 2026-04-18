#ifndef GENERATED_TYPES_H
#define GENERATED_TYPES_H

#define MSG_LIST(BTYPE, CTYPE, TTYPE, FIELD, ARRAY, SEQUENCE)                                                          \
    CTYPE (ros_Duration, "builtin_interfaces::msg::dds_::Duration",                                                    \
           "e8d009f659816f758b75334ee1a9ca5b5c0b859843261f14c7f937349599d93b",                                         \
           FIELD (int32_t, sec) FIELD (uint32_t, nanosec))                                                             \
    CTYPE (ros_Time, "builtin_interfaces::msg::dds_::Time",                                                            \
           "b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197",                                         \
           FIELD (int32_t, sec) FIELD (uint32_t, nanosec))                                                             \
    CTYPE (ros_Log, "rcl_interfaces::msg::dds_::Log",                                                                  \
           "e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac",                                         \
           FIELD (ros_Time, stamp) FIELD (uint8_t, level) FIELD (rstring, name) FIELD (rstring, msg)                   \
               FIELD (rstring, file) FIELD (rstring, function) FIELD (uint32_t, line))                                 \
    BTYPE (ros_String, "std_msgs::msg::dds_::String",                                                                  \
           "df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18", rstring)

#define SRV_LIST(SRV, REQUEST, REPLY, FIELD, ARRAY, SEQUENCE)

#endif /* GENERATED_TYPES_H */
