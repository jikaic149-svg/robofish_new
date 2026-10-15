#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

class DeviceMutex {
 public:
  bool begin() {
    if (!handle_) handle_ = xSemaphoreCreateMutexStatic(&storage_);
    return handle_ != nullptr;
  }

  bool take(TickType_t timeout) {
    return handle_ && xSemaphoreTake(handle_, timeout) == pdTRUE;
  }

  void give() {
    if (handle_) xSemaphoreGive(handle_);
  }

 private:
  SemaphoreHandle_t handle_ = nullptr;
  StaticSemaphore_t storage_{};
};

class ScopedDeviceLock {
 public:
  ScopedDeviceLock(DeviceMutex &mutex, TickType_t timeout)
      : mutex_(mutex), acquired_(mutex_.take(timeout)) {}

  ~ScopedDeviceLock() {
    if (acquired_) mutex_.give();
  }

  explicit operator bool() const { return acquired_; }

 private:
  DeviceMutex &mutex_;
  bool acquired_;
};
