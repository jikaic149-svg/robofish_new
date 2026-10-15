#include "robot_link.h"
#include <string.h>
#include <stdatomic.h>
#include "bsp_pin_config.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define COMMAND_QUEUE_LENGTH 4U
#define COMMAND_ACK_MS 100U
static StaticQueue_t s_command_queue_storage, s_status_queue_storage;
static uint8_t s_command_bytes[COMMAND_QUEUE_LENGTH * sizeof(rf_spi_command_t)];
static uint8_t s_status_bytes[sizeof(robot_link_status_t)];
static QueueHandle_t s_commands, s_status;
static StaticTask_t s_task_storage;
static StackType_t s_stack[4096 / sizeof(StackType_t)];
static spi_device_handle_t s_device;
static DMA_ATTR uint8_t s_tx[RF_SPI_FRAME_BYTES], s_rx[RF_SPI_FRAME_BYTES];
static portMUX_TYPE s_id_lock=portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_next_command=1;
static uint32_t s_session;
static _Atomic uint32_t s_completed_at_us;
static void IRAM_ATTR transaction_complete(spi_transaction_t *transaction) {
    (void)transaction;
    atomic_store_explicit(&s_completed_at_us, (uint32_t)esp_timer_get_time(), memory_order_relaxed);
}

static void link_task(void *unused) {
    (void)unused;
    robot_link_status_t status={0};
    spi_transaction_t transaction={.length=RF_SPI_FRAME_BYTES*8U,
        .rxlength=RF_SPI_FRAME_BYTES*8U,.tx_buffer=s_tx,.rx_buffer=s_rx};
    uint32_t history_sequence[8]={0}, history_time[8]={0};
    uint32_t sequence=0, outstanding=0, command_sent_ms=0;
    bool in_flight=false, completed_once=false;
    status.battery_state=RF_SPI_BATTERY_UNAVAILABLE;
    TickType_t wake=xTaskGetTickCount();
    for (;;) {
        uint32_t now_ms=(uint32_t)(esp_timer_get_time()/1000);
        if (in_flight) {
            spi_transaction_t *finished=NULL;
            esp_err_t err=spi_device_get_trans_result(s_device,&finished,0);
            if (err==ESP_OK) {
                in_flight=false; completed_once=true;
                if (finished==&transaction && rf_spi_valid(s_rx,sizeof(s_rx),RF_SPI_RESPONSE) &&
                    rf_spi_u32(s_rx+8)==s_session &&
                    rf_spi_after(rf_spi_u32(s_rx+16),status.peer_ack)) {
                    uint32_t received_us=(uint32_t)esp_timer_get_time();
                    status.received++; status.last_receive_ms=now_ms;
                    status.peer_sequence=rf_spi_u32(s_rx+12);
                    status.peer_ack=rf_spi_u32(s_rx+16);
                    status.peer_time_us=rf_spi_u32(s_rx+20);
                    unsigned slot=status.peer_ack % 8U;
                    if (history_sequence[slot]==status.peer_ack && status.peer_ack) {
                        status.round_trip_us=received_us-history_time[slot];
                        status.clock_offset_us=(int32_t)(status.peer_time_us-
                            (history_time[slot]+status.round_trip_us/2U));
                    }
                    status.ready=(s_rx[5]&RF_SPI_READY)!=0;
                    status.command_ack=rf_spi_u32(s_rx+24);
                    if (outstanding && status.command_ack==outstanding) {
                        status.command_result=s_rx[28]; outstanding=0;
                    }
                    status.mode=s_rx[29]; status.action=s_rx[30];
                    status.power=s_rx[31]; status.strength=s_rx[32];
                    status.failsafe=s_rx[33]; status.source=s_rx[34];
                    status.battery_state=s_rx[35];
                    status.frame_attempts=rf_spi_u32(s_rx+36);
                    status.frame_failures=rf_spi_u32(s_rx+40);
                    status.tail1=(int16_t)rf_spi_u16(s_rx+64);
                    status.tail2=(int16_t)rf_spi_u16(s_rx+66);
                    status.left_fin=(int16_t)rf_spi_u16(s_rx+68);
                    status.right_fin=(int16_t)rf_spi_u16(s_rx+70);
                    status.left_esc=(int16_t)rf_spi_u16(s_rx+72);
                    status.right_esc=(int16_t)rf_spi_u16(s_rx+74);
                    status.pitch=(int16_t)rf_spi_u16(s_rx+76);
                    status.wifi_control_allowed=(s_rx[5]&RF_SPI_WIFI_CONTROL_ALLOWED)!=0;
                    status.battery_valid=s_rx[61]!=0;
                    status.battery_age_ms=rf_spi_u16(s_rx+62);
                    status.battery_mv=rf_spi_u16(s_rx+78);
                } else status.invalid++;
            } else if (err!=ESP_ERR_TIMEOUT) { status.errors++; }
        }
        bool was_online=status.online;
        status.online=status.received &&
            (uint32_t)(now_ms-status.last_receive_ms)<RF_SPI_OFFLINE_MS;
        if (!status.online) { status.ready=false; status.battery_valid=false;
            status.battery_state=RF_SPI_BATTERY_UNAVAILABLE;
            status.wifi_control_allowed=false; }
        if (was_online && !status.online) xQueueReset(s_commands);
        if (outstanding && (uint32_t)(now_ms-command_sent_ms)>=COMMAND_ACK_MS) {
            status.command_result=RF_SPI_RESULT_TIMEOUT; outstanding=0;
        }
        uint32_t sent_us=(uint32_t)esp_timer_get_time();
        uint32_t completed_us=atomic_load_explicit(&s_completed_at_us, memory_order_relaxed);
        if (!in_flight && (!completed_once ||
            (uint32_t)(sent_us-completed_us)>=RF_SPI_MIN_CS_HIGH_US)) {
            if (++sequence==0) sequence=1;
            rf_spi_begin(s_tx,RF_SPI_REQUEST,s_session,sequence,status.peer_sequence,sent_us);
            if (!outstanding && status.ready && status.wifi_control_allowed) {
                rf_spi_command_t command;
                if (xQueueReceive(s_commands,&command,0)==pdTRUE) {
                    rf_spi_command_write(s_tx,&command);
                    outstanding=command.id; command_sent_ms=now_ms;
                    status.command_id=command.id; status.command_result=0;
                }
            }
            rf_spi_seal(s_tx);
            unsigned slot=sequence%8U;
            history_sequence[slot]=sequence; history_time[slot]=sent_us;
            esp_err_t err=spi_device_queue_trans(s_device,&transaction,0);
            if (err==ESP_OK) { in_flight=true; status.sent++; }
            else {
                status.errors++;
                if (outstanding && status.command_id==outstanding) {
                    status.command_result=RF_SPI_RESULT_TIMEOUT; outstanding=0;
                }
            }
        }
        xQueueOverwrite(s_status,&status);
        /* Late scheduling skips catch-up bursts so the slave has a CS-high gap. */
        TickType_t current=xTaskGetTickCount();
        if ((TickType_t)(current-wake)>=pdMS_TO_TICKS(RF_SPI_PERIOD_MS)) wake=current;
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(RF_SPI_PERIOD_MS));
    }
}
esp_err_t robot_link_start(void) {
    if (s_commands) return ESP_ERR_INVALID_STATE;
    spi_bus_config_t bus={.mosi_io_num=BSP_PIN_SPI_MOSI,
        .miso_io_num=BSP_PIN_SPI_MISO,.sclk_io_num=BSP_PIN_SPI_CLOCK,
        .quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=RF_SPI_FRAME_BYTES,
        .isr_cpu_id=ESP_INTR_CPU_AFFINITY_1};
    esp_err_t err=spi_bus_initialize(SPI2_HOST,&bus,SPI_DMA_CH_AUTO);
    if (err!=ESP_OK) return err;
    spi_device_interface_config_t device={.clock_speed_hz=RF_SPI_CLOCK_HZ,
        .mode=0,.spics_io_num=BSP_PIN_SPI_CS,.queue_size=1,.cs_ena_posttrans=2,
        .post_cb=transaction_complete};
    err=spi_bus_add_device(SPI2_HOST,&device,&s_device);
    if (err!=ESP_OK) { spi_bus_free(SPI2_HOST); return err; }
    s_commands=xQueueCreateStatic(COMMAND_QUEUE_LENGTH,sizeof(rf_spi_command_t),
        s_command_bytes,&s_command_queue_storage);
    s_status=xQueueCreateStatic(1,sizeof(robot_link_status_t),s_status_bytes,&s_status_queue_storage);
    s_session=esp_random(); if (!s_session) s_session=1;
    if (!xTaskCreateStaticPinnedToCore(link_task,"RobotSPI",sizeof(s_stack),NULL,5,
            s_stack,&s_task_storage,1)) {
        vQueueDelete(s_commands); vQueueDelete(s_status); s_commands=NULL; s_status=NULL;
        spi_bus_remove_device(s_device); spi_bus_free(SPI2_HOST);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
bool robot_link_snapshot(robot_link_status_t *status) {
    return status && s_status && xQueuePeek(s_status,status,0)==pdTRUE;
}
esp_err_t robot_link_submit(const rf_spi_command_t *request, uint32_t *id) {
    if (!request || !id || !rf_spi_command_valid(request)) return ESP_ERR_INVALID_ARG;
    robot_link_status_t state;
    if (!robot_link_snapshot(&state) || !state.online || !state.ready || !state.wifi_control_allowed)
        return ESP_ERR_INVALID_STATE;
    rf_spi_command_t command=*request;
    portENTER_CRITICAL(&s_id_lock);
    command.id=s_next_command++; if (!s_next_command) s_next_command=1;
    portEXIT_CRITICAL(&s_id_lock);
    if (xQueueSend(s_commands,&command,0)!=pdTRUE) return ESP_ERR_NO_MEM;
    *id=command.id; return ESP_OK;
}
