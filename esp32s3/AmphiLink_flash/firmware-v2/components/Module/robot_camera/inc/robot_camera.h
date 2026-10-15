#ifndef ROBOT_CAMERA_H
#define ROBOT_CAMERA_H
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t robot_camera_register(httpd_handle_t server);
bool robot_camera_available(void);
#ifdef __cplusplus
}
#endif
#endif
