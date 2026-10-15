#pragma once
#include <atomic>
#include <stdint.h>

namespace robofish {
// Tasks publish only completed work. Neither RF traffic nor output success is health.
struct TaskHealth {
  std::atomic<uint32_t> lastDone{0};
  std::atomic_bool seen{false};
  void complete(uint32_t now) {
    lastDone.store(now, std::memory_order_relaxed);
    seen.store(true, std::memory_order_release);
  }
  bool healthy(uint32_t now, uint32_t deadline) const {
    return seen.load(std::memory_order_acquire) &&
           uint32_t(now - lastDone.load(std::memory_order_relaxed)) < deadline;
  }
};
static_assert(std::atomic<uint32_t>::is_always_lock_free, "Health publication must not block");
}
