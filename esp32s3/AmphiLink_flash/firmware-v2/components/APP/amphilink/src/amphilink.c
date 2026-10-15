#include "amphilink.h"

#include "esp_err.h"
#include "esp_ota_ops.h"

#include "dap_core_service.h"
#include "flash_user.h"
#include "transport_manager.h"
#include "robot_link.h"

void amphilink_start(void)
{
    ESP_ERROR_CHECK(flash_user_init());
    ESP_ERROR_CHECK(dap_core_service_init());
    ESP_ERROR_CHECK(robot_link_start());
    ESP_ERROR_CHECK(transport_manager_start());
    const esp_partition_t *running=esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running && esp_ota_get_state_partition(running,&state)==ESP_OK &&
        state==ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
    }
}
