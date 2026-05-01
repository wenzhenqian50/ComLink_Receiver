#ifndef COMLINK_PROTOCOL_H
#define COMLINK_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NOW_PKT_BROADCAST = 0,
    NOW_PKT_PAIR_REQ,
    NOW_PKT_PAIR_ACK,
    NOW_PKT_P2P_DATA,
    NOW_PKT_FEEDBACK,
} now_pkt_type_t;

typedef struct __attribute__((packed)) {
    int16_t joy_x;
    int16_t joy_y;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    uint16_t buttons;
    uint32_t timestamp;
} controller_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t battery_level;
    uint8_t haptic_request;
    uint16_t data;
} feedback_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t pkt_type;
    union {
        controller_packet_t controller;
        feedback_packet_t feedback;
    } payload;
} comlink_now_pkt_t;

#ifdef __cplusplus
}
#endif

#endif
