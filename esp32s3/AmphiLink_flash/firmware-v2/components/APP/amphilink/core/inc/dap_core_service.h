#ifndef DAP_CORE_SERVICE_H
#define DAP_CORE_SERVICE_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define DEBUG_PACKET_SIZE 64U

typedef struct {
    size_t size;
    uint8_t data[DEBUG_PACKET_SIZE];
} debug_packet_t;

typedef enum { DAP_LINK_NONE=0, DAP_LINK_USB=1, DAP_LINK_TCP=2 } dap_link_t;
bool dap_core_service_busy(void);
bool dap_core_service_network_begin(void);
void dap_core_service_network_end(void *context);
esp_err_t dap_core_service_init(void);
esp_err_t dap_core_service_reset(dap_link_t link);

/* Called by the active transport receive path. This function never blocks. */
esp_err_t dap_core_service_submit(dap_link_t link, const uint8_t *data, size_t size);

/* Called only by the active transport task. */
esp_err_t dap_core_service_receive(dap_link_t link, debug_packet_t *packet, TickType_t wait_ticks);

#endif
