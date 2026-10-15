#include "bsp_system.h"

#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "bsp_pin_config.h"
#include "soc/rtc_cntl_reg.h"

void bsp_system_get_unique_id(uint8_t id[6])
{
    esp_read_mac(id, ESP_MAC_WIFI_STA);
}

uint32_t bsp_system_get_timestamp_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

void bsp_system_delay_us(uint32_t delay_us)
{
    esp_rom_delay_us(delay_us);
}

void bsp_system_enter_download(void)
{
    gpio_set_direction((gpio_num_t)BSP_PIN_TARGET_RESET, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)BSP_PIN_TARGET_RESET);
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
    __builtin_unreachable();
}
