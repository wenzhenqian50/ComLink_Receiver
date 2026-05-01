#ifndef COMLINK_CRSF_H
#define COMLINK_CRSF_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "comlink_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMLINK_CRSF_DEFAULT_UART UART_NUM_1
#define COMLINK_CRSF_DEFAULT_TX_GPIO GPIO_NUM_17
#define COMLINK_CRSF_DEFAULT_RX_GPIO GPIO_NUM_18
#define COMLINK_CRSF_DEFAULT_BAUD 420000

typedef struct {
    uart_port_t uart_num;
    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;
    int baud_rate;
    uint32_t output_period_ms;
    uint32_t failsafe_timeout_ms;
} comlink_crsf_config_t;

esp_err_t comlink_crsf_init(const comlink_crsf_config_t *config);
void comlink_crsf_update_control(const controller_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif
