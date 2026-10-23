#pragma once
#include <stdint.h>

#pragma pack(push, 1)
struct AttitudePacket {
    uint8_t start1;
    uint8_t start2;
    float w;
    float x;
    float y;
    float z;
};
#pragma pack(pop)
