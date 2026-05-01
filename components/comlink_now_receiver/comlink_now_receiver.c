#include "comlink_now_receiver.h"

#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "COMLINK_NOW";

static comlink_now_receiver_config_t s_config;
static uint8_t s_locked_mac[6];
static bool s_is_locked;
static TaskHandle_t s_feedback_task;
static uint8_t s_print_counter;

static esp_err_t send_feedback_to_controller(void)
{
    if (!s_is_locked) {
        return ESP_OK;
    }

    comlink_now_pkt_t tx_pkt = {0};
    const uint8_t intensity = s_config.feedback_haptic_intensity & 0x0F;
    const uint8_t duration = s_config.feedback_haptic_duration & 0x0F;

    tx_pkt.pkt_type = NOW_PKT_FEEDBACK;
    tx_pkt.payload.feedback.battery_level = s_config.feedback_battery_level;
    tx_pkt.payload.feedback.haptic_request = (duration << 4) | intensity;

    return esp_now_send(s_locked_mac, (const uint8_t *)&tx_pkt, sizeof(tx_pkt));
}

static void feedback_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(s_config.feedback_period_ms);

    while (1) {
        vTaskDelay(period);
        esp_err_t err = send_feedback_to_controller();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Feedback send failed: %s", esp_err_to_name(err));
        }
    }
}

static void send_pair_ack(const uint8_t peer_mac[6])
{
    comlink_now_pkt_t ack_pkt = {
        .pkt_type = NOW_PKT_PAIR_ACK,
    };

    esp_err_t err = esp_now_send(peer_mac, (const uint8_t *)&ack_pkt, sizeof(ack_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Pair ACK send failed: %s", esp_err_to_name(err));
    }
}

static void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "ESP-NOW sent to " MACSTR, MAC2STR(tx_info->des_addr));
    }
}

static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len != sizeof(comlink_now_pkt_t)) {
        return;
    }

    comlink_now_pkt_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    const uint8_t *src_mac = recv_info->src_addr;

    switch (pkt.pkt_type) {
    case NOW_PKT_BROADCAST:
        if (++s_print_counter >= 20) {
            ESP_LOGI(TAG, "Broadcast from " MACSTR " joy=(%d,%d) buttons=0x%04x",
                     MAC2STR(src_mac),
                     pkt.payload.controller.joy_x,
                     pkt.payload.controller.joy_y,
                     pkt.payload.controller.buttons);
            s_print_counter = 0;
        }
        break;

    case NOW_PKT_PAIR_REQ:
        if (!s_is_locked) {
            ESP_LOGI(TAG, "Pairing request from " MACSTR, MAC2STR(src_mac));
            memcpy(s_locked_mac, src_mac, sizeof(s_locked_mac));

            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, s_locked_mac, sizeof(peer.peer_addr));
            peer.channel = 0;
            peer.encrypt = false;

            if (!esp_now_is_peer_exist(s_locked_mac)) {
                esp_err_t err = esp_now_add_peer(&peer);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Add peer failed: %s", esp_err_to_name(err));
                    return;
                }
            }

            s_is_locked = true;
            send_pair_ack(s_locked_mac);
            ESP_LOGI(TAG, "P2P locked to " MACSTR, MAC2STR(s_locked_mac));
        } else if (memcmp(src_mac, s_locked_mac, sizeof(s_locked_mac)) == 0) {
            send_pair_ack(s_locked_mac);
        }
        break;

    case NOW_PKT_P2P_DATA:
        if (s_is_locked && memcmp(src_mac, s_locked_mac, sizeof(s_locked_mac)) == 0) {
            if (s_config.on_control != NULL) {
                s_config.on_control(&pkt.payload.controller, src_mac, s_config.user_ctx);
            }
        }
        break;

    default:
        break;
    }
}

esp_err_t comlink_now_receiver_init(const comlink_now_receiver_config_t *config)
{
    memset(&s_config, 0, sizeof(s_config));
    if (config != NULL) {
        s_config = *config;
    }

    if (s_config.feedback_period_ms == 0) {
        s_config.feedback_period_ms = 1000;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (s_config.channel > 0) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(s_config.channel, WIFI_SECOND_CHAN_NONE));
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    if (s_feedback_task == NULL) {
        BaseType_t task_ok = xTaskCreate(feedback_task,
                                         "comlink_feedback",
                                         2048,
                                         NULL,
                                         4,
                                         &s_feedback_task);
        if (task_ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
    return ESP_OK;
}

bool comlink_now_receiver_is_locked(void)
{
    return s_is_locked;
}
