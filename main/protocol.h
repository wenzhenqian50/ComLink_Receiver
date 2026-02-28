#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 通讯类型
typedef enum {
    NOW_PKT_BROADCAST = 0, // 广播模式: 广播数据包
    NOW_PKT_PAIR_REQ,      // P2P 模式：配对请求
    NOW_PKT_PAIR_ACK,      // P2P 模式：接收端确认
    NOW_PKT_P2P_DATA,      // P2P 模式：锁定后的点对点数据
    NOW_PKT_FEEDBACK,      // P2P 模式: 接收机回传
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

// 回传数据包
typedef struct __attribute__((packed)) {
    uint8_t battery_level;  // 接收端电量
    uint8_t haptic_request; // 让手柄震动 (低四位强度 高四位时长)
    uint16_t data;          // 其他数据
} feedback_packet_t;

// 包结构
typedef struct __attribute__((packed)) {
    uint8_t pkt_type;                   // 包类型
    union {
        controller_packet_t controller; // 手柄发给接收端
        feedback_packet_t feedback;     // 接收端发给手柄
    } payload;
} comlink_now_pkt_t;

#ifdef __cplusplus
}
#endif

#endif