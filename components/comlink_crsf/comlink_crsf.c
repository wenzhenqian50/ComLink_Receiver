#include "comlink_crsf.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_RC_CHANNEL_COUNT 16
#define CRSF_RC_PAYLOAD_SIZE 22
#define CRSF_RC_FRAME_SIZE 26
#define CRSF_RC_FRAME_LENGTH 24
#define CRSF_CRC_POLY 0xD5

#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1811
#define CRSF_BUTTON_AXIS_BIAS 200
#define CRSF_YAW_AXIS_SCALE_PERCENT 50

#define COMLINK_CRSF_SAFE_FRAME_COUNT 5
#define COMLINK_CRSF_TASK_STACK_SIZE 3072
#define COMLINK_CRSF_TASK_PRIORITY 8

#define BUTTON_ROLL_LEFT BIT0
#define BUTTON_ROLL_RIGHT BIT1
#define BUTTON_PITCH_DOWN BIT2
#define BUTTON_PITCH_UP BIT3
#define BUTTON_AUX1_LATCH BIT4
#define BUTTON_AUX2_LATCH BIT5
#define BUTTON_AUX3_LATCH BIT6
#define BUTTON_LATCH_MASK (BUTTON_AUX1_LATCH | BUTTON_AUX2_LATCH | BUTTON_AUX3_LATCH)

static const char *TAG = "COMLINK_CRSF";

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static comlink_crsf_config_t s_config;
static TaskHandle_t s_crsf_task;
static controller_packet_t s_latest_packet;
static TickType_t s_last_update_tick;
static uint16_t s_previous_buttons;
static uint16_t s_latched_buttons;
static bool s_seen_control;
static bool s_link_was_lost = true;

static int16_t clamp_i16(int16_t value, int16_t min, int16_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static int32_t clamp_i32(int32_t value, int32_t min, int32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static uint16_t map_positive_throttle(int16_t axis)
{
    if (axis <= 0) {
        return CRSF_CHANNEL_MIN;
    }

    int32_t value = axis;
    if (value > 32767) {
        value = 32767;
    }

    return (uint16_t)(CRSF_CHANNEL_MIN +
                      (value * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 32767);
}

static uint16_t map_signed_axis(int16_t axis)
{
    const int32_t value = clamp_i16(axis, -32767, 32767);

    if (value >= 0) {
        return (uint16_t)(CRSF_CHANNEL_MID +
                          (value * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)) / 32767);
    }

    return (uint16_t)(CRSF_CHANNEL_MID +
                      (value * (CRSF_CHANNEL_MID - CRSF_CHANNEL_MIN)) / 32767);
}

static uint16_t map_scaled_signed_axis(int16_t axis, int32_t scale_percent)
{
    int32_t scaled_axis = ((int32_t)axis * scale_percent) / 100;
    scaled_axis = clamp_i32(scaled_axis, -32767, 32767);
    return map_signed_axis((int16_t)scaled_axis);
}

static uint16_t map_button_axis(bool low_pressed, bool high_pressed)
{
    if (low_pressed == high_pressed) {
        return CRSF_CHANNEL_MID;
    }

    return high_pressed
               ? (CRSF_CHANNEL_MID + CRSF_BUTTON_AXIS_BIAS)
               : (CRSF_CHANNEL_MID - CRSF_BUTTON_AXIS_BIAS);
}

static void build_channels(const controller_packet_t *packet,
                           uint16_t latched_buttons,
                           uint16_t channels[CRSF_RC_CHANNEL_COUNT])
{
    memset(channels, 0, sizeof(uint16_t) * CRSF_RC_CHANNEL_COUNT);

    channels[0] = map_button_axis((packet->buttons & BUTTON_ROLL_LEFT) != 0,
                                  (packet->buttons & BUTTON_ROLL_RIGHT) != 0);
    channels[1] = map_button_axis((packet->buttons & BUTTON_PITCH_DOWN) != 0,
                                  (packet->buttons & BUTTON_PITCH_UP) != 0);
    channels[2] = map_positive_throttle(packet->joy_x);
    channels[3] = map_scaled_signed_axis(packet->joy_y, CRSF_YAW_AXIS_SCALE_PERCENT);
    channels[4] = (latched_buttons & BUTTON_AUX1_LATCH) ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    channels[5] = (latched_buttons & BUTTON_AUX2_LATCH) ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    channels[6] = (latched_buttons & BUTTON_AUX3_LATCH) ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;

    for (int i = 7; i < CRSF_RC_CHANNEL_COUNT; i++) {
        channels[i] = CRSF_CHANNEL_MIN;
    }
}

static void build_safe_channels(uint16_t channels[CRSF_RC_CHANNEL_COUNT])
{
    channels[0] = CRSF_CHANNEL_MID;
    channels[1] = CRSF_CHANNEL_MID;
    channels[2] = CRSF_CHANNEL_MIN;
    channels[3] = CRSF_CHANNEL_MID;

    for (int i = 4; i < CRSF_RC_CHANNEL_COUNT; i++) {
        channels[i] = CRSF_CHANNEL_MIN;
    }
}

static void pack_channels(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                          uint8_t payload[CRSF_RC_PAYLOAD_SIZE])
{
    memset(payload, 0, CRSF_RC_PAYLOAD_SIZE);

    uint16_t bit_index = 0;
    for (int ch = 0; ch < CRSF_RC_CHANNEL_COUNT; ch++) {
        uint16_t value = channels[ch] & 0x07FF;
        for (int bit = 0; bit < 11; bit++) {
            if ((value & (1U << bit)) != 0) {
                payload[bit_index >> 3] |= (uint8_t)(1U << (bit_index & 0x07));
            }
            bit_index++;
        }
    }
}

static uint8_t crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ CRSF_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

static void build_rc_frame(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                           uint8_t frame[CRSF_RC_FRAME_SIZE])
{
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = CRSF_RC_FRAME_LENGTH;
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    pack_channels(channels, &frame[3]);
    frame[CRSF_RC_FRAME_SIZE - 1] = crc8_dvb_s2(&frame[2], CRSF_RC_FRAME_LENGTH - 1);
}

static void send_channels(const uint16_t channels[CRSF_RC_CHANNEL_COUNT])
{
    uint8_t frame[CRSF_RC_FRAME_SIZE];
    build_rc_frame(channels, frame);
    (void)uart_write_bytes(s_config.uart_num, (const char *)frame, sizeof(frame));
}

static void mark_link_lost(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_latched_buttons = 0;
    s_link_was_lost = true;
    portEXIT_CRITICAL(&s_state_lock);
}

static void crsf_output_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(s_config.output_period_ms);
    const TickType_t timeout = pdMS_TO_TICKS(s_config.failsafe_timeout_ms);
    TickType_t last_wake = xTaskGetTickCount();
    bool silent = true;
    int safe_frames_remaining = 0;

    while (1) {
        controller_packet_t packet;
        TickType_t last_update_tick;
        uint16_t latched_buttons;
        bool seen_control;

        portENTER_CRITICAL(&s_state_lock);
        packet = s_latest_packet;
        last_update_tick = s_last_update_tick;
        latched_buttons = s_latched_buttons;
        seen_control = s_seen_control;
        portEXIT_CRITICAL(&s_state_lock);

        const TickType_t now = xTaskGetTickCount();
        const bool link_fresh = seen_control && ((now - last_update_tick) <= timeout);

        if (link_fresh) {
            uint16_t channels[CRSF_RC_CHANNEL_COUNT];
            build_channels(&packet, latched_buttons, channels);
            send_channels(channels);
            silent = false;
            safe_frames_remaining = 0;
        } else {
            if (!silent && safe_frames_remaining == 0) {
                ESP_LOGW(TAG, "Control link timeout, sending safe CRSF frames");
                mark_link_lost();
                safe_frames_remaining = COMLINK_CRSF_SAFE_FRAME_COUNT;
            }

            if (safe_frames_remaining > 0) {
                uint16_t channels[CRSF_RC_CHANNEL_COUNT];
                build_safe_channels(channels);
                send_channels(channels);
                safe_frames_remaining--;
                if (safe_frames_remaining == 0) {
                    silent = true;
                    ESP_LOGW(TAG, "CRSF output stopped for Betaflight RXLOSS");
                }
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t comlink_crsf_init(const comlink_crsf_config_t *config)
{
    if (s_crsf_task != NULL) {
        return ESP_OK;
    }

    s_config = (comlink_crsf_config_t) {
        .uart_num = COMLINK_CRSF_DEFAULT_UART,
        .tx_gpio = COMLINK_CRSF_DEFAULT_TX_GPIO,
        .rx_gpio = COMLINK_CRSF_DEFAULT_RX_GPIO,
        .baud_rate = COMLINK_CRSF_DEFAULT_BAUD,
        .output_period_ms = 10,
        .failsafe_timeout_ms = 100,
    };

    if (config != NULL) {
        s_config = *config;
    }
    if (s_config.output_period_ms == 0) {
        s_config.output_period_ms = 10;
    }
    if (s_config.failsafe_timeout_ms == 0) {
        s_config.failsafe_timeout_ms = 100;
    }

    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(s_config.uart_num, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_config.uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_config.uart_num,
                                 s_config.tx_gpio,
                                 s_config.rx_gpio,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(s_config.uart_num, UART_SIGNAL_INV_DISABLE));

    BaseType_t task_ok = xTaskCreate(crsf_output_task,
                                     "comlink_crsf",
                                     COMLINK_CRSF_TASK_STACK_SIZE,
                                     NULL,
                                     COMLINK_CRSF_TASK_PRIORITY,
                                     &s_crsf_task);
    if (task_ok != pdPASS) {
        uart_driver_delete(s_config.uart_num);
        s_crsf_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "CRSF UART%d TX=%d RX=%d baud=%d initialized",
             s_config.uart_num,
             s_config.tx_gpio,
             s_config.rx_gpio,
             s_config.baud_rate);
    return ESP_OK;
}

void comlink_crsf_update_control(const controller_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_state_lock);

    if (!s_seen_control || s_link_was_lost) {
        s_latched_buttons = 0;
        s_previous_buttons = packet->buttons;
        s_link_was_lost = false;
    } else {
        const uint16_t rising_edges = (uint16_t)(packet->buttons & ~s_previous_buttons);
        s_latched_buttons ^= (rising_edges & BUTTON_LATCH_MASK);
        s_previous_buttons = packet->buttons;
    }

    s_latest_packet = *packet;
    s_last_update_tick = xTaskGetTickCount();
    s_seen_control = true;

    portEXIT_CRITICAL(&s_state_lock);
}
