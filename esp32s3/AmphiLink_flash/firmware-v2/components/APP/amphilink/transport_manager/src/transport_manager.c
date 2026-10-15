#include "transport_manager.h"
#include <stdbool.h>
#include "esp_log.h"
#include "tcp_transport.h"
#include "usb_transport.h"

static bool s_usb_started;
static bool s_tcp_started;

esp_err_t transport_manager_start(void)
{
    if (s_usb_started && s_tcp_started) return ESP_ERR_INVALID_STATE;
    if (!s_usb_started) {
        esp_err_t err = usb_transport_start();
        if (err != ESP_OK) return err;
        s_usb_started = true;
    }
    if (!s_tcp_started) {
        esp_err_t err = tcp_transport_start();
        if (err != ESP_OK) {
            ESP_LOGE("TRANSPORT_MANAGER", "Wireless startup failed; USB remains available: %s", esp_err_to_name(err));
            return err;
        }
        s_tcp_started = true;
    }
    return ESP_OK;
}
