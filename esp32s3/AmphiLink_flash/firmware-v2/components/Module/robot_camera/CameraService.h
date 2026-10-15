#pragma once

#include <atomic>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

// Unknown /control variables are delegated to the application so the camera
// parameter protocol can coexist with RoboFish motion commands on one route.
using CameraControlFallback = esp_err_t (*)(httpd_req_t *request,
                                            const char *variable, int value,
                                            void *context);

class CameraService {
 public:
  CameraService();

  // Initializes the XIAO ESP32-S3 Sense camera and GPIO21 fill light using
  // the configured XIAO ESP32-S3 Sense hardware mapping. Returns
  // ESP_ERR_NOT_SUPPORTED when the local esp32-camera component is absent.
  esp_err_t begin();
  bool available() const;

  void setControlFallback(CameraControlFallback fallback, void *context);

  // Registers /status, /control, /capture and /bmp on an existing main HTTP
  // server. Call handleControlRequest() from an application-owned combined
  // /control route instead if that URI is already registered.
  esp_err_t registerControlRoutes(httpd_handle_t server,
                                  bool includeControlRoute = true);
  esp_err_t handleControlRequest(httpd_req_t *request);

  // Starts the recovered independent MJPEG /stream server on controlPort+1.
  esp_err_t startStreamServer(uint16_t controlPort = 80);
  esp_err_t stop();

 private:
  static esp_err_t statusHandler(httpd_req_t *request);
  static esp_err_t controlHandler(httpd_req_t *request);
  static esp_err_t captureHandler(httpd_req_t *request);
  static esp_err_t bmpHandler(httpd_req_t *request);
  static esp_err_t streamHandler(httpd_req_t *request);
  static esp_err_t xclkHandler(httpd_req_t *request);
  static esp_err_t registerWriteHandler(httpd_req_t *request);
  static esp_err_t registerReadHandler(httpd_req_t *request);
  static esp_err_t pllHandler(httpd_req_t *request);
  static esp_err_t resolutionHandler(httpd_req_t *request);

  esp_err_t sendStatus(httpd_req_t *request);
  esp_err_t sendCapture(httpd_req_t *request);
  esp_err_t sendBmp(httpd_req_t *request);
  esp_err_t sendStream(httpd_req_t *request);
  esp_err_t setXclk(httpd_req_t *request);
  esp_err_t writeRegister(httpd_req_t *request);
  esp_err_t readRegister(httpd_req_t *request);
  esp_err_t setPll(httpd_req_t *request);
  esp_err_t setResolution(httpd_req_t *request);
  esp_err_t dispatchControl(httpd_req_t *request, const char *variable,
                            int value);
  esp_err_t setFillLight(uint8_t intensity, bool forceOff = false);

  httpd_handle_t streamServer_ = nullptr;
  CameraControlFallback controlFallback_ = nullptr;
  void *controlFallbackContext_ = nullptr;
  std::atomic_bool initialized_{false};
  std::atomic_bool streaming_{false};
  std::atomic_uint_fast16_t lightIntensity_{0};
  void *lightChannel_ = nullptr;
};

// True only when esp_camera.h and img_converters.h were visible while this
// translation unit was compiled. The ESP-IDF 5.3.2 framework itself does not
// ship esp32-camera; it must be provided as a local project component.
bool roboFishCameraDriverCompiled();
