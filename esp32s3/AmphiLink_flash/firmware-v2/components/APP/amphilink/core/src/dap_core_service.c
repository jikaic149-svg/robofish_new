/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The CMSIS-DAP v2 command queue behavior implemented here follows the ARM
 * CMSIS-DAP firmware contract and is adapted to ESP-IDF/FreeRTOS.
 * Upstream: https://github.com/ARM-software/CMSIS-DAP
 * Local license copy: referance_project_licenses/CMSIS-DAP/LICENSE
 */

#include "dap_core_service.h"

#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "DAP.h"
#include "DAP_config.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

_Static_assert(DAP_PACKET_COUNT > 0U, "Request and response queues must not be empty");
_Static_assert(DAP_PACKET_SIZE == DEBUG_PACKET_SIZE,
               "Core and transport packet sizes must match");

typedef enum {
    CORE_EVENT_REQUEST,
    CORE_EVENT_RESET,
} core_event_type_t;

typedef struct {
    core_event_type_t type;
    dap_link_t link;
    unsigned int session;
    debug_packet_t packet;
} core_event_t;

static const char *TAG = "DAP_CORE";
static QueueHandle_t s_request_queue;
static QueueHandle_t s_response_queue[3];
static StaticQueue_t s_response_control[3];
static uint8_t s_response_storage[3][DAP_PACKET_COUNT * sizeof(debug_packet_t)];
/* Low two bits: owner; upper bits: generation to invalidate queued old commands. */
static atomic_uint s_session;
bool dap_core_service_busy(void) { unsigned owner = atomic_load(&s_session) & 3U; return owner == DAP_LINK_USB || owner == DAP_LINK_TCP; }
bool dap_core_service_network_begin(void)
{
    unsigned current = atomic_load(&s_session);
    if ((current & 3U) != DAP_LINK_NONE) return false;
    return atomic_compare_exchange_strong(&s_session, &current, current | 3U);
}
void dap_core_service_network_end(void *context)
{
    (void)context;
    unsigned current = atomic_load(&s_session);
    if ((current & 3U) == 3U) atomic_compare_exchange_strong(&s_session, &current, (current & ~3U) + 4U);
}
static SemaphoreHandle_t s_reset_mutex;
static SemaphoreHandle_t s_reset_complete;
static TaskHandle_t s_core_task;

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define DAP_CORE_TASK_CORE 1
#else
#define DAP_CORE_TASK_CORE 0
#endif

static void dap_core_execute_request(debug_packet_t *request, dap_link_t link, unsigned session)
{
    debug_packet_t response = {
        .size = 0U,
        .data = {0},
    };
    if (request->data[0] != ID_DAP_Info && atomic_load(&s_session) != session) return;
    uint32_t result = DAP_ExecuteCommand(request->data, response.data);
    response.size = (uint16_t)result;

    if ((response.size == 0U) || (response.size > sizeof(response.data))) {
        ESP_LOGE(TAG, "Invalid response size: %u", (unsigned int)response.size);
        return;
    }

    if (xQueueSend(s_response_queue[link], &response, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue response");
    }
    if (request->data[0] == ID_DAP_Disconnect) {
        unsigned expected = session; atomic_compare_exchange_strong(&s_session, &expected, (session & ~3U) + 4U);
    }
}

static void dap_core_apply_reset(void)
{
    DAP_Setup();
    DAP_TransferAbort = 0U;
    xSemaphoreGive(s_reset_complete);
}

static void dap_core_task(void *argument)
{
    (void)argument;
    core_event_t batch[DAP_PACKET_COUNT];

    while (true) {
        core_event_t event;
        if (xQueueReceive(s_request_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == CORE_EVENT_RESET) {
            dap_core_apply_reset();
            continue;
        }

        if (event.packet.data[0] != ID_DAP_QueueCommands) {
            dap_core_execute_request(&event.packet, event.link, event.session);
            continue;
        }

        size_t batch_count = 0U;
        bool reset_requested = false;
        do {
            batch[batch_count++] = event;
            if (event.packet.data[0] != ID_DAP_QueueCommands) {
                break;
            }
            if (batch_count == DAP_PACKET_COUNT) {
                ESP_LOGE(TAG, "Queued command sequence has no execute packet");
                break;
            }
            if (xQueueReceive(s_request_queue, &event, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            if (event.type == CORE_EVENT_RESET) {
                reset_requested = true;
                break;
            }
        } while (true);

        if (reset_requested) {
            dap_core_apply_reset();
            continue;
        }

        for (size_t index = 0U; index < batch_count; ++index) {
            if (batch[index].packet.data[0] == ID_DAP_QueueCommands) {
                batch[index].packet.data[0] = ID_DAP_ExecuteCommands;
            }
            dap_core_execute_request(&batch[index].packet, batch[index].link, batch[index].session);
        }
    }
}

static void dap_core_service_cleanup(void)
{
    if (s_reset_complete != NULL) {
        vSemaphoreDelete(s_reset_complete);
        s_reset_complete = NULL;
    }
    if (s_reset_mutex != NULL) {
        vSemaphoreDelete(s_reset_mutex);
        s_reset_mutex = NULL;
    }
    for (int i=1; i<=2; ++i) { if (s_response_queue[i]) { vQueueDelete(s_response_queue[i]); s_response_queue[i]=NULL; } }
    if (s_request_queue != NULL) {
        vQueueDelete(s_request_queue);
        s_request_queue = NULL;
    }
}

esp_err_t dap_core_service_init(void)
{
    if (s_request_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_request_queue = xQueueCreate(DAP_PACKET_COUNT, sizeof(core_event_t));
    for (int i=1; i<=2; ++i) s_response_queue[i] = xQueueCreateStatic(DAP_PACKET_COUNT, sizeof(debug_packet_t), s_response_storage[i], &s_response_control[i]);
    atomic_store(&s_session, 0U);
    s_reset_mutex = xSemaphoreCreateMutex();
    s_reset_complete = xSemaphoreCreateBinary();
    if ((s_request_queue == NULL) || (s_response_queue[1] == NULL || s_response_queue[2] == NULL) ||
        (s_reset_mutex == NULL) || (s_reset_complete == NULL)) {
        dap_core_service_cleanup();
        return ESP_ERR_NO_MEM;
    }

    DAP_Setup();
    DAP_TransferAbort = 0U;

    if (xTaskCreatePinnedToCore(dap_core_task, "DAP-Core", 4096, NULL,
                                configMAX_PRIORITIES - 2, &s_core_task,
                                DAP_CORE_TASK_CORE) != pdPASS) {
        dap_core_service_cleanup();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Core task and packet queues initialized");
    return ESP_OK;
}

esp_err_t dap_core_service_reset(dap_link_t link)
{
    if ((s_core_task == NULL) || (s_reset_mutex == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (link != DAP_LINK_USB && link != DAP_LINK_TCP) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_reset_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return ESP_ERR_TIMEOUT;
    unsigned session = atomic_load(&s_session);
    if ((session & 3U) != (unsigned)link) {
        xQueueReset(s_response_queue[link]);
        xSemaphoreGive(s_reset_mutex);
        return ESP_OK;
    }
    DAP_TransferAbort = 1U;

    xQueueReset(s_request_queue);
    xQueueReset(s_response_queue[link]);
    while (xSemaphoreTake(s_reset_complete, 0) == pdTRUE) {
    }

    const core_event_t reset_event = {
        .type = CORE_EVENT_RESET,
        .packet = {
            .size = 0U,
            .data = {0},
        },
    };

    esp_err_t err = ESP_OK;
    if (xQueueSendToFront(s_request_queue, &reset_event, pdMS_TO_TICKS(100)) != pdTRUE) {
        err = ESP_FAIL;
    } else if (xSemaphoreTake(s_reset_complete, pdMS_TO_TICKS(500)) != pdTRUE) {
        err = ESP_FAIL;
    }

    /* Discard a response produced by a request that was executing during reset. */
    xQueueReset(s_response_queue[link]);
    if (err == ESP_OK) atomic_compare_exchange_strong(&s_session, &session, (session & ~3U) + 4U);
    xSemaphoreGive(s_reset_mutex);
    return err;
}

esp_err_t dap_core_service_submit(dap_link_t link, const uint8_t *data, size_t size)
{
    if ((data == NULL) || (size == 0U) || (size > DEBUG_PACKET_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_request_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (link != DAP_LINK_USB && link != DAP_LINK_TCP) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_reset_mutex, 0) != pdTRUE) return ESP_ERR_TIMEOUT;
    unsigned session = atomic_load(&s_session);
    unsigned owner = session & 3U;
    bool info_only = data[0] == ID_DAP_Info;
    if (data[0] == ID_DAP_TransferAbort && owner != link) {
        xSemaphoreGive(s_reset_mutex);
        return ESP_OK;
    }
    if (!info_only && owner != DAP_LINK_NONE && owner != link) {
        /* CMSIS-DAP has no generic BUSY code. Reject Connect with disabled port,
         * and other commands with Invalid, without touching the active target. */
        debug_packet_t rejected = {.size = 1U, .data = {ID_DAP_Invalid}};
        if (data[0] == ID_DAP_Connect) {
            rejected.size = 2U;
            rejected.data[0] = ID_DAP_Connect;
            rejected.data[1] = 0U;
        }
        bool sent = xQueueSend(s_response_queue[link], &rejected, 0) == pdTRUE;
        xSemaphoreGive(s_reset_mutex);
        return sent ? ESP_OK : ESP_ERR_TIMEOUT;
    }
    if (!info_only && owner == DAP_LINK_NONE) {
        unsigned desired = session | (unsigned)link;
        if (!atomic_compare_exchange_strong(&s_session, &session, desired)) {
            xSemaphoreGive(s_reset_mutex); return ESP_ERR_INVALID_STATE;
        }
        session = desired;
    }
    if (data[0] == ID_DAP_TransferAbort) {
        DAP_TransferAbort = 1U;
        xSemaphoreGive(s_reset_mutex);
        return ESP_OK;
    }

    core_event_t event = {
        .type = CORE_EVENT_REQUEST,
        .link = link,
        .session = session,
        .packet = {
            .size = size,
            .data = {0},
        },
    };
    memcpy(event.packet.data, data, size);

    bool sent = xQueueSend(s_request_queue, &event, 0) == pdTRUE;
    if (!sent && !info_only && owner == DAP_LINK_NONE) atomic_compare_exchange_strong(&s_session, &session, (session & ~3U) + 4U);
    xSemaphoreGive(s_reset_mutex);
    return sent ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t dap_core_service_receive(dap_link_t link, debug_packet_t *packet, TickType_t wait_ticks)
{
    if (packet == NULL || (link != DAP_LINK_USB && link != DAP_LINK_TCP)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_response_queue[1] == NULL || s_response_queue[2] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xQueueReceive(s_response_queue[link], packet, wait_ticks) == pdTRUE)
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}
