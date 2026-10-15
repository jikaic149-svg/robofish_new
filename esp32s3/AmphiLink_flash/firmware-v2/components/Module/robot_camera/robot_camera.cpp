#include "robot_camera.h"
#include "CameraService.h"
#include "esp_log.h"
namespace { CameraService camera; }
extern "C" bool robot_camera_available(void) { return camera.available(); }
extern "C" esp_err_t robot_camera_register(httpd_handle_t server) {
    esp_err_t initialized=camera.begin();
    if (initialized!=ESP_OK)
        ESP_LOGW("robot_camera","Camera unavailable: %s",esp_err_to_name(initialized));
    esp_err_t err=camera.registerControlRoutes(server);
    if (err!=ESP_OK) return err;
    if (initialized==ESP_OK) return camera.startStreamServer(80);
    return ESP_OK;
}
