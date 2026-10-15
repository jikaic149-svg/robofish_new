#ifndef BSP_PIN_CONFIG_H
#define BSP_PIN_CONFIG_H

// Board signal routing for ESP32-S3.
#define BSP_PIN_DEBUG_CLOCK          4U
#define BSP_PIN_DEBUG_DATA           5U
/* PCB2: XIAO D5/GPIO6 routes to CN1.1 and STM32 NRST. */
#define BSP_PIN_TARGET_RESET         6U
#define BSP_PIN_STATUS_CONNECTED_LED 42U
#define BSP_PIN_STATUS_RUNNING_LED   41U
#define BSP_PIN_MODESEL              7U

#define BSP_PIN_SPI_CS               1
#define BSP_PIN_SPI_CLOCK            2
#define BSP_PIN_SPI_MOSI             43
#define BSP_PIN_SPI_MISO             44

#endif
