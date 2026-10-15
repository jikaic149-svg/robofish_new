#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "spi_link_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    bool online, ready, wifi_control_allowed, battery_valid;
    uint16_t battery_mv, battery_age_ms;
    uint32_t sent, received, invalid, errors, last_receive_ms;
    uint32_t peer_sequence, peer_ack, peer_time_us, round_trip_us;
    int32_t clock_offset_us;
    uint32_t command_id, command_ack, frame_attempts, frame_failures;
    uint8_t command_result, mode, action, power, strength, failsafe, source, battery_state;
    int16_t tail1, tail2, left_fin, right_fin, left_esc, right_esc, pitch;
} robot_link_status_t;
esp_err_t robot_link_start(void);
bool robot_link_snapshot(robot_link_status_t *status);
esp_err_t robot_link_submit(const rf_spi_command_t *command, uint32_t *id);
#ifdef __cplusplus
}
#endif
#endif
