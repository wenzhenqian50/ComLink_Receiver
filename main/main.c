#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "protocol.h"

static const char *TAG = "RECEIVER";

static uint8_t locked_mac[6] = {0}; // 手柄 MAC
static bool is_p2p_locked = false;  // 是否已锁定

static void send_feedback_to_controller() {
    if (!is_p2p_locked) return; // 没绑定手柄就不发

    comlink_now_pkt_t tx_pkt;
    uint8_t intensity = 10; // 0-15 级
    uint8_t duration = 4;    // 0-15 级
    tx_pkt.pkt_type = NOW_PKT_FEEDBACK;
    tx_pkt.payload.feedback.battery_level = 85;
    tx_pkt.payload.feedback.haptic_request = (duration << 4) | (intensity & 0x0F);

    // 发送回手柄的 MAC 地址
    esp_err_t res = esp_now_send(locked_mac, (uint8_t *)&tx_pkt, sizeof(tx_pkt));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Feedback send failed");
    }
}


/* 发送回调：确认 ACK 是否成功发给手柄 */
static void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "ACK sent successfully to " MACSTR, MAC2STR(tx_info->des_addr));
    }
}

/* 接收回调：处理两种模式 */
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len != sizeof(comlink_now_pkt_t)) return;

    comlink_now_pkt_t *pkt = (comlink_now_pkt_t *)data;
    const uint8_t *src_mac = recv_info->src_addr;

    switch (pkt->pkt_type) {
        case NOW_PKT_BROADCAST:
            // 处理广播数据
            ESP_LOGI(TAG, "[BROADCAST] From: " MACSTR " | Joy_X:%d, Joy_Y:%d, Pitch:%d, Roll:%d, Yaw:%d, Buttons:0x%X\n", MAC2STR(src_mac), pkt->payload.controller.joy_x, pkt->payload.controller.joy_y, pkt->payload.controller.pitch, pkt->payload.controller.roll, pkt->payload.controller.yaw, pkt->payload.controller.buttons);
            break;
        case NOW_PKT_PAIR_REQ:
            // 处理配对请求
            if (!is_p2p_locked) {
                ESP_LOGW(TAG, "Pairing request from " MACSTR ". Locking session...", MAC2STR(src_mac));
                
                // 记录手柄 MAC 地址
                memcpy(locked_mac, src_mac, 6);
                
                // 注册为 Peer
                esp_now_peer_info_t peer = {};
                memcpy(peer.peer_addr, locked_mac, 6);
                peer.channel = 0; // 使用当前信道
                peer.encrypt = false;
                
                if (!esp_now_is_peer_exist(locked_mac)) {
                    esp_now_add_peer(&peer);
                }

                // 回发配对响应包
                comlink_now_pkt_t ack_pkt = { .pkt_type = NOW_PKT_PAIR_ACK };
                esp_now_send(locked_mac, (uint8_t *)&ack_pkt, sizeof(ack_pkt));
                
                is_p2p_locked = true;
            }
            break;
        case NOW_PKT_P2P_DATA:
            // 处理私有点对点数据
            if (is_p2p_locked && memcmp(src_mac, locked_mac, 6) == 0) {
                ESP_LOGI(TAG, "Joy_X:%d, Joy_Y:%d, Pitch:%d, Roll:%d, Yaw:%d, Buttons:0x%X\n", 
                         pkt->payload.controller.joy_x, pkt->payload.controller.joy_y, pkt->payload.controller.pitch, pkt->payload.controller.roll, pkt->payload.controller.yaw, pkt->payload.controller.buttons);
            }
            break;
        default:
            break;
    }
}

void wifi_init_receiver(void) {
    // 初始化基础栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 初始化 Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 固定信道 与手柄一致
    esp_wifi_set_channel(0, WIFI_SECOND_CHAN_NONE);

    // 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    ESP_LOGI(TAG, "Receiver Initialized. Waiting for broadcast or pairing...");
}

void app_main(void) {
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_receiver();

    while (1) {
        send_feedback_to_controller();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}