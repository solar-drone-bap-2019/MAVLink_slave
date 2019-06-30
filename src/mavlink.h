// Taken from https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566
// Alternatively https://github.com/tmaxxdd/arduino-with-mavlink/

#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX 254
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#include "common/common.h"
#include "protocol.h"


#endif // MAVLINK_H