#include "CameraService.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_log.h"

#ifndef RF_HAS_ESP32_CAMERA
#define RF_HAS_ESP32_CAMERA 0
#endif

#if RF_HAS_ESP32_CAMERA
#define RF_CAMERA_DRIVER_COMPILED 1
#include "driver/sdm.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#else
#define RF_CAMERA_DRIVER_COMPILED 0
#endif

namespace {

constexpr char kTag[] = "camera";
constexpr int kCameraXclkHz = 12000000;
constexpr int kFillLightGpio = 21;
constexpr uint32_t kFillLightCarrierHz = 1000000;
constexpr size_t kMaxQueryBytes = 192;
constexpr char kStreamContentType[] =
    "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
constexpr char kStreamBoundary[] =
    "\r\n--123456789000000000000987654321\r\n";
constexpr char kStreamPartHeader[] =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n"
    "X-Timestamp: %d.%06d\r\n\r\n";

CameraService *serviceFrom(httpd_req_t *request) {
  return request ? static_cast<CameraService *>(request->user_ctx) : nullptr;
}

esp_err_t sendUnavailable(httpd_req_t *request) {
  if (!request) return ESP_ERR_INVALID_ARG;
  httpd_resp_set_status(request, "503 Service Unavailable");
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(
      request,
      "{\"error\":\"camera unavailable: local esp32-camera component is "
      "required\"}");
}

bool parseInteger(const char *text, int *value) {
  if (!text || !value || text[0] == '\0') return false;
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < INT_MIN ||
      parsed > INT_MAX) {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool isCameraSensorVariable(const char *variable) {
  if (!variable) return false;
  constexpr const char *kVariables[] = {
      "framesize",      "quality",   "contrast",       "brightness",
      "saturation",     "gainceiling", "colorbar",     "hmirror",
      "vflip",          "awb",       "agc",            "aec",
      "awb_gain",       "agc_gain",  "aec_value",      "aec2",
      "dcw",            "bpc",       "wpc",            "raw_gma",
      "lenc",           "special_effect", "wb_mode",    "ae_level",
  };
  return std::any_of(std::begin(kVariables), std::end(kVariables),
                     [variable](const char *known) {
                       return std::strcmp(variable, known) == 0;
                     });
}

esp_err_t parseControlQuery(httpd_req_t *request, char *variable,
                            size_t variableSize, int *value) {
  if (!request || !variable || variableSize == 0 || !value) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t queryLength = httpd_req_get_url_query_len(request);
  if (queryLength == 0 || queryLength >= kMaxQueryBytes) {
    return ESP_ERR_INVALID_ARG;
  }

  std::array<char, kMaxQueryBytes> query = {};
  std::array<char, 16> valueText = {};
  if (httpd_req_get_url_query_str(request, query.data(), query.size()) !=
          ESP_OK ||
      httpd_query_key_value(query.data(), "var", variable, variableSize) !=
          ESP_OK ||
      httpd_query_key_value(query.data(), "val", valueText.data(),
                            valueText.size()) != ESP_OK ||
      !parseInteger(valueText.data(), value)) {
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

#if RF_CAMERA_DRIVER_COMPILED

esp_err_t readQuery(httpd_req_t *request,
                    std::array<char, kMaxQueryBytes> *query) {
  if (!request || !query) return ESP_ERR_INVALID_ARG;
  const size_t length = httpd_req_get_url_query_len(request);
  if (length == 0 || length >= query->size()) return ESP_ERR_INVALID_ARG;
  query->fill('\0');
  return httpd_req_get_url_query_str(request, query->data(), query->size());
}

bool queryInteger(const char *query, const char *key, int *value) {
  if (!query || !key || !value) return false;
  std::array<char, 16> text = {};
  return httpd_query_key_value(query, key, text.data(), text.size()) == ESP_OK &&
         parseInteger(text.data(), value);
}

bool queryIntegerList(const char *query, const char *const *keys,
                      size_t keyCount, int *values) {
  if (!query || !keys || !values) return false;
  for (size_t index = 0; index < keyCount; ++index) {
    if (!queryInteger(query, keys[index], &values[index])) return false;
  }
  return true;
}

esp_err_t sendInvalidQuery(httpd_req_t *request) {
  httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                      "missing or invalid camera query");
  return ESP_OK;
}

esp_err_t sendCameraOperationResult(httpd_req_t *request, int result) {
  if (result != 0) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "camera operation failed");
    return ESP_OK;
  }
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(request, nullptr, 0);
}

#endif

esp_err_t registerRoute(httpd_handle_t server, const char *uri,
                        esp_err_t (*handler)(httpd_req_t *), void *context) {
  const httpd_uri_t route = {
      .uri = uri,
      .method = HTTP_GET,
      .handler = handler,
      .user_ctx = context,
  };
  return httpd_register_uri_handler(server, &route);
}

#if RF_CAMERA_DRIVER_COMPILED

sdm_channel_handle_t lightHandle(void *opaque) {
  return static_cast<sdm_channel_handle_t>(opaque);
}

esp_err_t setSensorParameter(sensor_t *sensor, const char *variable,
                             int value) {
  if (!sensor || !variable) return ESP_ERR_INVALID_ARG;

  int result = -1;
  if (std::strcmp(variable, "framesize") == 0) {
    if (sensor->pixformat == PIXFORMAT_JPEG) {
      result = sensor->set_framesize(sensor,
                                     static_cast<framesize_t>(value));
    }
  } else if (std::strcmp(variable, "quality") == 0) {
    result = sensor->set_quality(sensor, value);
  } else if (std::strcmp(variable, "contrast") == 0) {
    result = sensor->set_contrast(sensor, value);
  } else if (std::strcmp(variable, "brightness") == 0) {
    result = sensor->set_brightness(sensor, value);
  } else if (std::strcmp(variable, "saturation") == 0) {
    result = sensor->set_saturation(sensor, value);
  } else if (std::strcmp(variable, "gainceiling") == 0) {
    result = sensor->set_gainceiling(sensor,
                                     static_cast<gainceiling_t>(value));
  } else if (std::strcmp(variable, "colorbar") == 0) {
    result = sensor->set_colorbar(sensor, value);
  } else if (std::strcmp(variable, "hmirror") == 0) {
    result = sensor->set_hmirror(sensor, value);
  } else if (std::strcmp(variable, "vflip") == 0) {
    result = sensor->set_vflip(sensor, value);
  } else if (std::strcmp(variable, "awb") == 0) {
    result = sensor->set_whitebal(sensor, value);
  } else if (std::strcmp(variable, "agc") == 0) {
    result = sensor->set_gain_ctrl(sensor, value);
  } else if (std::strcmp(variable, "aec") == 0) {
    result = sensor->set_exposure_ctrl(sensor, value);
  } else if (std::strcmp(variable, "awb_gain") == 0) {
    result = sensor->set_awb_gain(sensor, value);
  } else if (std::strcmp(variable, "agc_gain") == 0) {
    result = sensor->set_agc_gain(sensor, value);
  } else if (std::strcmp(variable, "aec_value") == 0) {
    result = sensor->set_aec_value(sensor, value);
  } else if (std::strcmp(variable, "aec2") == 0) {
    result = sensor->set_aec2(sensor, value);
  } else if (std::strcmp(variable, "dcw") == 0) {
    result = sensor->set_dcw(sensor, value);
  } else if (std::strcmp(variable, "bpc") == 0) {
    result = sensor->set_bpc(sensor, value);
  } else if (std::strcmp(variable, "wpc") == 0) {
    result = sensor->set_wpc(sensor, value);
  } else if (std::strcmp(variable, "raw_gma") == 0) {
    result = sensor->set_raw_gma(sensor, value);
  } else if (std::strcmp(variable, "lenc") == 0) {
    result = sensor->set_lenc(sensor, value);
  } else if (std::strcmp(variable, "special_effect") == 0) {
    result = sensor->set_special_effect(sensor, value);
  } else if (std::strcmp(variable, "wb_mode") == 0) {
    result = sensor->set_wb_mode(sensor, value);
  } else if (std::strcmp(variable, "ae_level") == 0) {
    result = sensor->set_ae_level(sensor, value);
  } else {
    return ESP_ERR_NOT_FOUND;
  }

  return result == 0 ? ESP_OK : ESP_FAIL;
}

class FrameGuard {
 public:
  explicit FrameGuard(camera_fb_t *frame) : frame_(frame) {}
  ~FrameGuard() {
    if (frame_) esp_camera_fb_return(frame_);
  }
  camera_fb_t *get() const { return frame_; }

 private:
  camera_fb_t *frame_;
};

class HeapBufferGuard {
 public:
  explicit HeapBufferGuard(uint8_t *buffer) : buffer_(buffer) {}
  ~HeapBufferGuard() { std::free(buffer_); }

 private:
  uint8_t *buffer_;
};

#endif

}  // namespace

// 生命周期与服务注册 --------------------------------------------------------

CameraService::CameraService() = default;

bool roboFishCameraDriverCompiled() {
  return RF_CAMERA_DRIVER_COMPILED != 0;
}

esp_err_t CameraService::begin() {
#if !RF_CAMERA_DRIVER_COMPILED
  ESP_LOGW(kTag,
           "Camera disabled: add the local esp32-camera component and its "
           "build dependency");
  return ESP_ERR_NOT_SUPPORTED;
#else
  if (initialized_.load(std::memory_order_acquire)) return ESP_OK;

  const bool hasPsram =
      heap_caps_get_total_size(MALLOC_CAP_SPIRAM) != 0;
  camera_config_t configuration = {};
  configuration.pin_pwdn = -1;
  configuration.pin_reset = -1;
  configuration.pin_xclk = 10;
  configuration.pin_sccb_sda = 40;
  configuration.pin_sccb_scl = 39;
  configuration.pin_d7 = 48;
  configuration.pin_d6 = 11;
  configuration.pin_d5 = 12;
  configuration.pin_d4 = 14;
  configuration.pin_d3 = 16;
  configuration.pin_d2 = 18;
  configuration.pin_d1 = 17;
  configuration.pin_d0 = 15;
  configuration.pin_vsync = 38;
  configuration.pin_href = 47;
  configuration.pin_pclk = 13;
  configuration.xclk_freq_hz = kCameraXclkHz;
  configuration.ledc_timer = LEDC_TIMER_0;
  // The migrated actuator BSP already owns channels 0..4. XCLK has no
  // electrical dependency on the original channel number, so channel 7
  // preserves the recovered 12 MHz signal without corrupting tail PWM.
  configuration.ledc_channel = LEDC_CHANNEL_7;
  configuration.pixel_format = PIXFORMAT_JPEG;
  // Preserve the raw enum values recovered from the exact image. They are
  // intentionally not translated to resolution names across camera versions.
  // UXGA needs a frame buffer that does not fit in internal DRAM. The current
  // recovered flash configuration intentionally leaves PSRAM disabled, so
  // initialize directly at the SVGA size used after sensor detection.
  configuration.frame_size =
      static_cast<framesize_t>(hasPsram ? 15 : 11);
  configuration.jpeg_quality = hasPsram ? 15 : 12;
  configuration.fb_count = hasPsram ? 2 : 1;
  configuration.fb_location =
      hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  configuration.grab_mode =
      hasPsram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t error = esp_camera_init(&configuration);
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "Camera initialization failed: %s",
             esp_err_to_name(error));
    return error;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) {
    esp_camera_deinit();
    return ESP_ERR_NOT_FOUND;
  }
#ifdef OV3660_PID
  if (sensor->id.PID == OV3660_PID) {
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, 1);
    sensor->set_saturation(sensor, -2);
  }
#endif
  if (configuration.pixel_format == PIXFORMAT_JPEG) {
    sensor->set_framesize(sensor, static_cast<framesize_t>(11));
  }

  const sdm_config_t lightConfiguration = {
      .gpio_num = kFillLightGpio,
      .clk_src = SDM_CLK_SRC_DEFAULT,
      .sample_rate_hz = kFillLightCarrierHz,
      .flags = {.invert_out = 0, .io_loop_back = 0},
  };
  sdm_channel_handle_t light = nullptr;
  error = sdm_new_channel(&lightConfiguration, &light);
  if (error == ESP_OK) error = sdm_channel_enable(light);
  if (error != ESP_OK) {
    if (light) sdm_del_channel(light);
    esp_camera_deinit();
    ESP_LOGE(kTag, "GPIO21 fill light initialization failed: %s",
             esp_err_to_name(error));
    return error;
  }
  lightChannel_ = light;
  initialized_.store(true, std::memory_order_release);
  setFillLight(0, true);
  ESP_LOGI(kTag, "Camera ready: JPEG, XCLK=%d Hz, fill light GPIO%d",
           kCameraXclkHz, kFillLightGpio);
  return ESP_OK;
#endif
}

bool CameraService::available() const {
  return initialized_.load(std::memory_order_acquire);
}

void CameraService::setControlFallback(CameraControlFallback fallback,
                                       void *context) {
  controlFallback_ = fallback;
  controlFallbackContext_ = context;
}

esp_err_t CameraService::registerControlRoutes(httpd_handle_t server,
                                               bool includeControlRoute) {
  if (!server) return ESP_ERR_INVALID_ARG;
  esp_err_t error =
      registerRoute(server, "/status", statusHandler, this);
  if (error == ESP_OK && includeControlRoute) {
    error = registerRoute(server, "/control", controlHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/capture", captureHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/bmp", bmpHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/xclk", xclkHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/reg", registerWriteHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/greg", registerReadHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/pll", pllHandler, this);
  }
  if (error == ESP_OK) {
    error = registerRoute(server, "/resolution", resolutionHandler, this);
  }
  return error;
}

esp_err_t CameraService::startStreamServer(uint16_t controlPort) {
  if (!available()) return ESP_ERR_NOT_SUPPORTED;
  if (streamServer_) return ESP_ERR_INVALID_STATE;
  if (controlPort == UINT16_MAX) return ESP_ERR_INVALID_ARG;

  httpd_config_t configuration = HTTPD_DEFAULT_CONFIG();
  configuration.server_port = static_cast<uint16_t>(controlPort + 1);
  configuration.ctrl_port = static_cast<uint16_t>(configuration.ctrl_port + 1);
  configuration.stack_size = 4096;
  configuration.max_uri_handlers = 1;
  configuration.max_open_sockets = 2;
  configuration.lru_purge_enable = true;
  esp_err_t error = httpd_start(&streamServer_, &configuration);
  if (error != ESP_OK) return error;

  error = registerRoute(streamServer_, "/stream", streamHandler, this);
  if (error != ESP_OK) {
    httpd_stop(streamServer_);
    streamServer_ = nullptr;
  }
  return error;
}

esp_err_t CameraService::stop() {
  esp_err_t result = ESP_OK;
  if (streamServer_) {
    result = httpd_stop(streamServer_);
    streamServer_ = nullptr;
  }

#if RF_CAMERA_DRIVER_COMPILED
  if (initialized_.exchange(false, std::memory_order_acq_rel)) {
    setFillLight(0, true);
    sdm_channel_handle_t light = lightHandle(lightChannel_);
    lightChannel_ = nullptr;
    if (light) {
      const esp_err_t disableError = sdm_channel_disable(light);
      if (result == ESP_OK) result = disableError;
      const esp_err_t deleteError = sdm_del_channel(light);
      if (result == ESP_OK) result = deleteError;
    }
    const esp_err_t cameraError = esp_camera_deinit();
    if (result == ESP_OK) result = cameraError;
  }
#endif
  return result;
}

// HTTP 路由入口 -------------------------------------------------------------

esp_err_t CameraService::statusHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->sendStatus(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::controlHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->handleControlRequest(request)
                 : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::captureHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->sendCapture(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::bmpHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->sendBmp(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::streamHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->sendStream(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::xclkHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->setXclk(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::registerWriteHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->writeRegister(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::registerReadHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->readRegister(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::pllHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->setPll(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t CameraService::resolutionHandler(httpd_req_t *request) {
  CameraService *service = serviceFrom(request);
  return service ? service->setResolution(request) : ESP_ERR_INVALID_ARG;
}

// 传感器调试接口 ------------------------------------------------------------

esp_err_t CameraService::setXclk(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  std::array<char, kMaxQueryBytes> query = {};
  int frequencyMhz = 0;
  if (readQuery(request, &query) != ESP_OK ||
      !queryInteger(query.data(), "xclk", &frequencyMhz) ||
      frequencyMhz < 1 || frequencyMhz > 40) {
    return sendInvalidQuery(request);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || !sensor->set_xclk) return sendUnavailable(request);
  return sendCameraOperationResult(
      request, sensor->set_xclk(sensor, LEDC_TIMER_0, frequencyMhz));
#endif
}

esp_err_t CameraService::writeRegister(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  std::array<char, kMaxQueryBytes> query = {};
  int reg = 0;
  int mask = 0;
  int value = 0;
  if (readQuery(request, &query) != ESP_OK ||
      !queryInteger(query.data(), "reg", &reg) ||
      !queryInteger(query.data(), "mask", &mask) ||
      !queryInteger(query.data(), "val", &value) || reg < 0 || reg > 0xffff ||
      mask < 0 || value < 0) {
    return sendInvalidQuery(request);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || !sensor->set_reg) return sendUnavailable(request);
  return sendCameraOperationResult(
      request, sensor->set_reg(sensor, reg, mask, value));
#endif
}

esp_err_t CameraService::readRegister(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  std::array<char, kMaxQueryBytes> query = {};
  int reg = 0;
  int mask = 0;
  if (readQuery(request, &query) != ESP_OK ||
      !queryInteger(query.data(), "reg", &reg) ||
      !queryInteger(query.data(), "mask", &mask) || reg < 0 ||
      reg > 0xffff || mask < 0) {
    return sendInvalidQuery(request);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || !sensor->get_reg) return sendUnavailable(request);
  const int value = sensor->get_reg(sensor, reg, mask);
  if (value < 0) return sendCameraOperationResult(request, value);

  std::array<char, 32> response = {};
  const int length = std::snprintf(response.data(), response.size(), "%d",
                                   value);
  httpd_resp_set_type(request, "text/plain");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(request, response.data(), length);
#endif
}

esp_err_t CameraService::setPll(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  constexpr const char *kKeys[] = {
      "bypass", "mul", "sys", "root", "pre", "seld5", "pclken", "pclk",
  };
  std::array<int, std::size(kKeys)> values = {};
  std::array<char, kMaxQueryBytes> query = {};
  if (readQuery(request, &query) != ESP_OK ||
      !queryIntegerList(query.data(), kKeys, std::size(kKeys),
                        values.data())) {
    return sendInvalidQuery(request);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || !sensor->set_pll) return sendUnavailable(request);
  return sendCameraOperationResult(
      request,
      sensor->set_pll(sensor, values[0], values[1], values[2], values[3],
                      values[4], values[5], values[6], values[7]));
#endif
}

esp_err_t CameraService::setResolution(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  constexpr const char *kKeys[] = {
      "sx", "sy", "ex", "ey", "offx", "offy", "tx", "ty", "ox", "oy",
      "scale", "binning",
  };
  std::array<int, std::size(kKeys)> values = {};
  std::array<char, kMaxQueryBytes> query = {};
  if (readQuery(request, &query) != ESP_OK ||
      !queryIntegerList(query.data(), kKeys, std::size(kKeys), values.data())) {
    return sendInvalidQuery(request);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || !sensor->set_res_raw) return sendUnavailable(request);
  return sendCameraOperationResult(
      request,
      sensor->set_res_raw(sensor, values[0], values[1], values[2], values[3],
                          values[4], values[5], values[6], values[7], values[8],
                          values[9], values[10], values[11]));
#endif
}

// 相机参数与补光控制 --------------------------------------------------------

esp_err_t CameraService::handleControlRequest(httpd_req_t *request) {
  std::array<char, 32> variable = {};
  int value = 0;
  if (parseControlQuery(request, variable.data(), variable.size(), &value) !=
      ESP_OK) {
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                        "missing or invalid var/val query");
    return ESP_OK;
  }
  return dispatchControl(request, variable.data(), value);
}

esp_err_t CameraService::dispatchControl(httpd_req_t *request,
                                         const char *variable, int value) {
  if (!request || !variable) return ESP_ERR_INVALID_ARG;

  esp_err_t error = ESP_ERR_NOT_FOUND;
  if (std::strcmp(variable, "led_intensity") == 0) {
    if (!available()) {
      error = ESP_ERR_NOT_SUPPORTED;
    } else if (value < 0 || value > 255) {
      error = ESP_ERR_INVALID_ARG;
    } else {
      lightIntensity_.store(static_cast<uint16_t>(value),
                            std::memory_order_relaxed);
      error = streaming_.load(std::memory_order_relaxed)
                  ? setFillLight(static_cast<uint8_t>(value))
                  : ESP_OK;
    }
  } else if (isCameraSensorVariable(variable)) {
#if RF_CAMERA_DRIVER_COMPILED
    if (available()) {
      error = setSensorParameter(esp_camera_sensor_get(), variable, value);
    } else {
      error = ESP_ERR_NOT_SUPPORTED;
    }
#else
    error = ESP_ERR_NOT_SUPPORTED;
#endif
  }

  if (error == ESP_ERR_NOT_FOUND && controlFallback_) {
    return controlFallback_(request, variable, value,
                            controlFallbackContext_);
  }
  if (error == ESP_ERR_NOT_FOUND) {
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                        "unknown control variable");
    return ESP_OK;
  }
  if (error == ESP_ERR_NOT_SUPPORTED) return sendUnavailable(request);
  if (error != ESP_OK) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "control update failed");
    return ESP_OK;
  }
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(request, nullptr, 0);
}

esp_err_t CameraService::setFillLight(uint8_t intensity, bool forceOff) {
#if !RF_CAMERA_DRIVER_COMPILED
  (void)intensity;
  (void)forceOff;
  return ESP_ERR_NOT_SUPPORTED;
#else
  sdm_channel_handle_t light = lightHandle(lightChannel_);
  if (!light) return ESP_ERR_INVALID_STATE;
  const int output = forceOff ? 0 : intensity;
  const int density = output == 0 ? -128 : output - 128;
  return sdm_channel_set_pulse_density(light, static_cast<int8_t>(density));
#endif
}

// 状态、抓拍与图传响应 ------------------------------------------------------

esp_err_t CameraService::sendStatus(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) return sendUnavailable(request);

  std::array<char, 768> response = {};
  const int length = std::snprintf(
      response.data(), response.size(),
      "{\"xclk\":%u,\"pixformat\":%u,\"framesize\":%u,"
      "\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
      "\"saturation\":%d,\"sharpness\":%d,\"special_effect\":%u,"
      "\"wb_mode\":%u,\"awb\":%u,\"awb_gain\":%u,\"aec\":%u,"
      "\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,\"agc\":%u,"
      "\"agc_gain\":%u,\"gainceiling\":%u,\"bpc\":%u,\"wpc\":%u,"
      "\"raw_gma\":%u,\"lenc\":%u,\"hmirror\":%u,\"dcw\":%u,"
      "\"vflip\":%u,\"colorbar\":%u,\"led_intensity\":%u}",
      static_cast<unsigned>(sensor->xclk_freq_hz),
      static_cast<unsigned>(sensor->pixformat),
      static_cast<unsigned>(sensor->status.framesize),
      static_cast<unsigned>(sensor->status.quality), sensor->status.brightness,
      sensor->status.contrast, sensor->status.saturation,
      sensor->status.sharpness,
      static_cast<unsigned>(sensor->status.special_effect),
      static_cast<unsigned>(sensor->status.wb_mode),
      static_cast<unsigned>(sensor->status.awb),
      static_cast<unsigned>(sensor->status.awb_gain),
      static_cast<unsigned>(sensor->status.aec),
      static_cast<unsigned>(sensor->status.aec2), sensor->status.ae_level,
      static_cast<unsigned>(sensor->status.aec_value),
      static_cast<unsigned>(sensor->status.agc),
      static_cast<unsigned>(sensor->status.agc_gain),
      static_cast<unsigned>(sensor->status.gainceiling),
      static_cast<unsigned>(sensor->status.bpc),
      static_cast<unsigned>(sensor->status.wpc),
      static_cast<unsigned>(sensor->status.raw_gma),
      static_cast<unsigned>(sensor->status.lenc),
      static_cast<unsigned>(sensor->status.hmirror),
      static_cast<unsigned>(sensor->status.dcw),
      static_cast<unsigned>(sensor->status.vflip),
      static_cast<unsigned>(sensor->status.colorbar),
      static_cast<unsigned>(lightIntensity_.load(std::memory_order_relaxed)));
  if (length <= 0 || static_cast<size_t>(length) >= response.size()) {
    return ESP_ERR_INVALID_SIZE;
  }
  httpd_resp_set_type(request, "application/json");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(request, response.data(), length);
#endif
}

esp_err_t CameraService::sendCapture(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  setFillLight(static_cast<uint8_t>(
      lightIntensity_.load(std::memory_order_relaxed)));
  camera_fb_t *frame = esp_camera_fb_get();
  setFillLight(0, true);
  if (!frame) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "camera capture failed");
    return ESP_OK;
  }
  FrameGuard frameGuard(frame);

  const uint8_t *payload = frame->buf;
  size_t payloadSize = frame->len;
  uint8_t *converted = nullptr;
  if (frame->format != PIXFORMAT_JPEG &&
      !frame2jpg(frame, 80, &converted, &payloadSize)) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "JPEG conversion failed");
    return ESP_OK;
  }
  HeapBufferGuard convertedGuard(converted);
  if (converted) payload = converted;

  std::array<char, 32> timestamp = {};
  std::snprintf(timestamp.data(), timestamp.size(), "%lld.%06ld",
                static_cast<long long>(frame->timestamp.tv_sec),
                static_cast<long>(frame->timestamp.tv_usec));
  httpd_resp_set_type(request, "image/jpeg");
  httpd_resp_set_hdr(request, "Content-Disposition",
                     "inline; filename=capture.jpg");
  httpd_resp_set_hdr(request, "X-Timestamp", timestamp.data());
  return httpd_resp_send(request,
                         reinterpret_cast<const char *>(payload),
                         static_cast<ssize_t>(payloadSize));
#endif
}

esp_err_t CameraService::sendBmp(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  setFillLight(static_cast<uint8_t>(
      lightIntensity_.load(std::memory_order_relaxed)));
  camera_fb_t *frame = esp_camera_fb_get();
  setFillLight(0, true);
  if (!frame) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "camera capture failed");
    return ESP_OK;
  }
  FrameGuard frameGuard(frame);

  uint8_t *bitmap = nullptr;
  size_t bitmapSize = 0;
  if (!frame2bmp(frame, &bitmap, &bitmapSize) || !bitmap) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "BMP conversion failed");
    return ESP_OK;
  }
  HeapBufferGuard bitmapGuard(bitmap);

  std::array<char, 32> timestamp = {};
  std::snprintf(timestamp.data(), timestamp.size(), "%lld.%06ld",
                static_cast<long long>(frame->timestamp.tv_sec),
                static_cast<long>(frame->timestamp.tv_usec));
  httpd_resp_set_type(request, "image/x-windows-bmp");
  httpd_resp_set_hdr(request, "Content-Disposition",
                     "inline; filename=capture.bmp");
  httpd_resp_set_hdr(request, "X-Timestamp", timestamp.data());
  return httpd_resp_send(request,
                         reinterpret_cast<const char *>(bitmap),
                         static_cast<ssize_t>(bitmapSize));
#endif
}

esp_err_t CameraService::sendStream(httpd_req_t *request) {
#if !RF_CAMERA_DRIVER_COMPILED
  return sendUnavailable(request);
#else
  if (!available()) return sendUnavailable(request);
  if (streaming_.exchange(true, std::memory_order_acq_rel)) {
    httpd_resp_set_status(request, "503 Service Unavailable");
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(request, "camera stream is already active");
  }

  httpd_resp_set_type(request, kStreamContentType);
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(request, "X-Framerate", "60");
  setFillLight(static_cast<uint8_t>(
      lightIntensity_.load(std::memory_order_relaxed)));

  esp_err_t error = ESP_OK;
  while (error == ESP_OK) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) {
      error = ESP_FAIL;
      break;
    }
    FrameGuard frameGuard(frame);

    const uint8_t *jpeg = frame->buf;
    size_t jpegSize = frame->len;
    uint8_t *converted = nullptr;
    if (frame->format != PIXFORMAT_JPEG &&
        !frame2jpg(frame, 80, &converted, &jpegSize)) {
      error = ESP_FAIL;
      break;
    }
    HeapBufferGuard convertedGuard(converted);
    if (converted) jpeg = converted;

    std::array<char, 128> partHeader = {};
    const int partHeaderLength = std::snprintf(
        partHeader.data(), partHeader.size(), kStreamPartHeader,
        static_cast<unsigned>(jpegSize),
        static_cast<int>(frame->timestamp.tv_sec),
        static_cast<int>(frame->timestamp.tv_usec));
    if (partHeaderLength <= 0 ||
        static_cast<size_t>(partHeaderLength) >= partHeader.size()) {
      error = ESP_ERR_INVALID_SIZE;
      break;
    }

    error = httpd_resp_send_chunk(request, kStreamBoundary,
                                  sizeof(kStreamBoundary) - 1);
    if (error == ESP_OK) {
      error = httpd_resp_send_chunk(request, partHeader.data(),
                                    partHeaderLength);
    }
    if (error == ESP_OK) {
      error = httpd_resp_send_chunk(
          request, reinterpret_cast<const char *>(jpeg),
          static_cast<ssize_t>(jpegSize));
    }
  }

  setFillLight(0, true);
  streaming_.store(false, std::memory_order_release);
  return error;
#endif
}
