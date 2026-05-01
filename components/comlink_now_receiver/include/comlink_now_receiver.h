#ifndef COMLINK_NOW_RECEIVER_H
#define COMLINK_NOW_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "comlink_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*comlink_now_control_cb_t)(const controller_packet_t *packet,
                                         const uint8_t src_mac[6],
                                         void *user_ctx);

typedef struct {
    comlink_now_control_cb_t on_control;
    void *user_ctx;
    uint8_t channel;
    uint8_t feedback_battery_level;
    uint8_t feedback_haptic_intensity;
    uint8_t feedback_haptic_duration;
    uint32_t feedback_period_ms;
} comlink_now_receiver_config_t;

esp_err_t comlink_now_receiver_init(const comlink_now_receiver_config_t *config);
bool comlink_now_receiver_is_locked(void);

#ifdef __cplusplus
}
#endif

#endif
