#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "comlink_crsf.h"
#include "comlink_now_receiver.h"

static const char *TAG = "MAIN";

static void handle_control_packet(const controller_packet_t *packet,
                                  const uint8_t src_mac[6],
                                  void *user_ctx)
{
    (void)src_mac;
    (void)user_ctx;
    comlink_crsf_update_control(packet);
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    init_nvs();

    ESP_ERROR_CHECK(comlink_crsf_init(NULL));

    const comlink_now_receiver_config_t now_config = {
        .on_control = handle_control_packet,
        .feedback_battery_level = 85,
        // Periodic haptic feedback is disabled to avoid continuous vibration.
        // .feedback_haptic_intensity = 10,
        // .feedback_haptic_duration = 4,
        .feedback_period_ms = 1000,
    };
    ESP_ERROR_CHECK(comlink_now_receiver_init(&now_config));

    ESP_LOGI(TAG, "ComLink receiver started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
