/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * The descriptor layout and Microsoft OS 2.0 handling are adapted from the
 * upstream TinyUSB WebUSB example for this CMSIS-DAP v2 device.
 * Upstream: https://github.com/hathach/tinyusb
 * Local license copy: referance_project_licenses/tinyusb/LICENSE
 */

#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "bsp_system.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "dap_usb_descriptors.h"
#include "esp_ota_ops.h"

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // Interface number, string index, EP OUT, EP IN, EP size
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_DAP, 4, EP_DAP_OUT, EP_DAP_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(USB_MS_OS_20_DESC_LEN, USB_MS_OS_VENDOR_CODE),
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

uint8_t const desc_ms_os_20[] = {
    // Microsoft OS 2.0 descriptor set header
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(USB_MS_OS_20_DESC_LEN),

    // Configuration and function subset headers
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0x00, 0x00, U16_TO_U8S_LE(USB_MS_OS_20_DESC_LEN - 0x0A),
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    ITF_NUM_DAP, 0x00, U16_TO_U8S_LE(USB_MS_OS_20_DESC_LEN - 0x12),

    // Bind the CMSIS-DAP vendor interface to WinUSB.
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Install the standard CMSIS-DAP v2 device interface GUID.
    U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00,
    'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00,
    'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00,
    0x00, 0x00, U16_TO_U8S_LE(0x0050),
    '{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00,
    'B', 0x00, '5', 0x00, 'A', 0x00, 'D', 0x00, '-', 0x00,
    '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00,
    '4', 0x00, '6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00,
    'A', 0x00, 'A', 0x00, '3', 0x00, '6', 0x00, '-', 0x00,
    '1', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '4', 0x00,
    '6', 0x00, '4', 0x00, '6', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == USB_MS_OS_20_DESC_LEN,
                 "Incorrect Microsoft OS 2.0 descriptor size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    if(request->bRequest==0x45U && request->bmRequestType==0xC0U &&
       request->wIndex==0xA11FU && request->wValue==0 && request->wLength==64) {
        static char version[64];
        if(stage!=CONTROL_STAGE_SETUP)return true;
        const esp_app_desc_t *desc=esp_app_get_description();
        memcpy(version,desc->version,32);memcpy(version+32,desc->project_name,32);
        return tud_control_xfer(rhport,request,version,sizeof(version));
    }
    if(request->bRequest==0x44U && request->bmRequestType==0xC0U &&
       request->wIndex==0xA11FU && request->wValue==0 && request->wLength==32) {
        static uint32_t info[8];
        if(stage!=CONTROL_STAGE_SETUP)return true;
        const esp_partition_t *part=esp_ota_get_running_partition();
        if(!part)return false;
        esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
        esp_ota_get_state_partition(part,&state);
        memcpy(info,"RLYU",4);info[1]=1;info[2]=part->address;info[3]=part->size;
        info[4]=(uint32_t)state;info[5]=0x58334432;info[6]=0x800000;info[7]=1;
        return tud_control_xfer(rhport,request,info,sizeof(info));
    }
    if (request->bRequest == 0x43U && request->bmRequestType == 0xC0U &&
        request->wIndex == 0xA11FU && (request->wValue == 3U || request->wValue == 6U) && request->wLength == 16U) {
        static uint8_t snapshot[16];
        if (stage != CONTROL_STAGE_SETUP) return true;
        const unsigned pin = request->wValue;
        memcpy(snapshot, "GPIN", 4);
        snapshot[4] = (GPIO.in >> pin) & 1U;
        snapshot[5] = (GPIO.out >> pin) & 1U;
        snapshot[6] = (GPIO.enable >> pin) & 1U;
        snapshot[7] = GPIO.pin[pin].pad_driver;
        uint32_t mux = REG_READ(IO_MUX_GPIO0_REG + 4U * pin);
        uint32_t matrix = GPIO.func_out_sel_cfg[pin].val;
        for (unsigned i = 0; i < 4; ++i) {
            snapshot[8+i] = (mux >> (8*i)) & 0xffU;
            snapshot[12+i] = (matrix >> (8*i)) & 0xffU;
        }
        return tud_control_xfer(rhport, request, snapshot, sizeof(snapshot));
    }
    /* USB-only maintenance protocol v1. Exact tuples prevent accidental
     * activation; these constants identify the command, not authentication.
     * ACK runs in the TinyUSB task after the status transaction completes.
     */
    if ((request->bRequest == 0x42U) &&
        (request->wIndex == 0xA11FU)) {
        static const uint8_t info[] = {'A', 'L', 'U', 'P', 1, 1, 0, 0};
        if ((request->bmRequestType == 0xC0U) &&
            (request->wValue == 0U) && (request->wLength == sizeof(info))) {
            return (stage == CONTROL_STAGE_SETUP)
                ? tud_control_xfer(rhport, request, (void *)info, sizeof(info))
                : true;
        }
        if ((request->bmRequestType == 0x40U) &&
            (request->wValue == 0xB007U) && (request->wLength == 0U)) {
            if (stage == CONTROL_STAGE_SETUP) {
                return tud_control_status(rhport, request);
            }
            if (stage == CONTROL_STAGE_ACK) {
                bsp_system_enter_download();
            }
            return true;
        }
        return false;
    }
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if ((request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) &&
        (request->bRequest == USB_MS_OS_VENDOR_CODE) &&
        (request->wIndex == 0x0007U)) {
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20,
                                sizeof(desc_ms_os_20));
    }

    return false;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    static uint16_t _desc_str[32];
    uint8_t chr_count = 0U;
    char serial[13];

    if (index == 0) {
        _desc_str[1] = 0x0409;
        chr_count = 1;
    } else {
        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char *str;
        if (index == USB_STRING_SERIAL_INDEX) {
            uint8_t id[6];
            bsp_system_get_unique_id(id);
            snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
                     id[0], id[1], id[2], id[3], id[4], id[5]);
            str = serial;
        } else {
            str = string_desc_arr[index];
        }
        if (str == NULL) {
            return NULL;
        }
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
