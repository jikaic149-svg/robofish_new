#ifndef ROBOFISH_SPI_LINK_PROTOCOL_H
#define ROBOFISH_SPI_LINK_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define RF_SPI_FRAME_BYTES 128U
#define RF_SPI_CRC_OFFSET 124U
#define RF_SPI_PERIOD_MS 1U
#define RF_SPI_OFFLINE_MS 100U
#define RF_SPI_CLOCK_HZ 8000000U
#define RF_SPI_MIN_CS_HIGH_US 100U
#define RF_SPI_BATTERY_NORMAL 0U
#define RF_SPI_BATTERY_LOW 1U
#define RF_SPI_BATTERY_HIGH 2U
#define RF_SPI_BATTERY_UNAVAILABLE 3U
#define RF_SPI_REQUEST 1U
#define RF_SPI_RESPONSE 2U
#define RF_SPI_READY 1U
#define RF_SPI_WIFI_CONTROL_ALLOWED 2U
#define RF_SPI_CMD_NONE 0U
#define RF_SPI_CMD_MOTION 1U
#define RF_SPI_CMD_LORA 2U
#define RF_SPI_CMD_PITCH 3U
/* Command type 4 is retired and must be rejected. */
#define RF_SPI_CMD_VISION 5U
#define RF_SPI_RESULT_ACCEPTED 1U
#define RF_SPI_RESULT_REJECTED 2U
#define RF_SPI_RESULT_TIMEOUT 3U

static inline uint16_t rf_spi_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t rf_spi_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void rf_spi_put16(uint8_t *p, uint16_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v >> 8);
}
static inline void rf_spi_put32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v >> 8);
    p[2]=(uint8_t)(v >> 16); p[3]=(uint8_t)(v >> 24);
}
static inline int rf_spi_after(uint32_t a, uint32_t b) {
    return a != b && (uint32_t)(a-b) < UINT32_C(0x80000000);
}
static inline uint32_t rf_spi_crc32(const uint8_t *p, size_t n) {
    uint32_t crc=UINT32_C(0xffffffff);
    while (n--) {
        crc ^= *p++;
        for (unsigned bit=0; bit<8; ++bit)
            crc=(crc >> 1) ^ (UINT32_C(0xedb88320) & (uint32_t)-(int32_t)(crc & 1U));
    }
    return ~crc;
}
static inline void rf_spi_begin(uint8_t *p, uint8_t kind, uint32_t session,
                               uint32_t sequence, uint32_t ack, uint32_t time_us) {
    memset(p,0,RF_SPI_FRAME_BYTES);
    p[0]='R'; p[1]='F'; p[2]='S'; p[3]='1';
    p[4]=kind;
    rf_spi_put16(p+6,kind == RF_SPI_REQUEST ? 24U : 56U);
    rf_spi_put32(p+8,session); rf_spi_put32(p+12,sequence);
    rf_spi_put32(p+16,ack); rf_spi_put32(p+20,time_us);
}
static inline void rf_spi_seal(uint8_t *p) {
    rf_spi_put32(p+RF_SPI_CRC_OFFSET,rf_spi_crc32(p,RF_SPI_CRC_OFFSET));
}
static inline int rf_spi_valid(const uint8_t *p, size_t n, uint8_t kind) {
    return p && n == RF_SPI_FRAME_BYTES && p[0]=='R' && p[1]=='F' &&
           p[2]=='S' && p[3]=='1' && p[4]==kind &&
           !(p[5] & (uint8_t)~(RF_SPI_READY | RF_SPI_WIFI_CONTROL_ALLOWED)) &&
           rf_spi_u16(p+6)==(kind == RF_SPI_REQUEST ? 24U : 56U) &&
           rf_spi_u32(p+RF_SPI_CRC_OFFSET)==rf_spi_crc32(p,RF_SPI_CRC_OFFSET);
}
/* Local command object; never transmitted as a C structure. */
typedef struct {
    uint32_t id;
    uint8_t type, action, mode, power, strength, code, has_knob;
    uint16_t knob;
    int16_t pitch, steering;
    uint8_t brake;
} rf_spi_command_t;
static inline void rf_spi_command_write(uint8_t *p, const rf_spi_command_t *c) {
    rf_spi_put32(p+24,c->id);
    p[28]=c->type; p[29]=c->action; p[30]=c->mode; p[31]=c->power;
    p[32]=c->strength; p[33]=c->code; p[34]=c->has_knob; p[35]=c->brake;
    rf_spi_put16(p+36,c->knob); rf_spi_put16(p+38,(uint16_t)c->pitch);
    rf_spi_put16(p+40,(uint16_t)c->steering);
}
static inline void rf_spi_command_read(const uint8_t *p, rf_spi_command_t *c) {
    memset(c,0,sizeof(*c));
    c->id=rf_spi_u32(p+24); c->type=p[28]; c->action=p[29];
    c->mode=p[30]; c->power=p[31]; c->strength=p[32];
    c->code=p[33]; c->has_knob=p[34]; c->brake=p[35];
    c->knob=rf_spi_u16(p+36); c->pitch=(int16_t)rf_spi_u16(p+38);
    c->steering=(int16_t)rf_spi_u16(p+40);
}
static inline int rf_spi_command_valid(const rf_spi_command_t *c) {
    if (!c || c->has_knob>1 || c->brake>1) return 0;
    switch (c->type) {
    case RF_SPI_CMD_MOTION: return c->action<=4 && c->mode<=3 &&
                                   c->power<=100 && c->strength<=100;
    case RF_SPI_CMD_LORA: return c->code<=21 && c->code!=8 &&
                                 c->code!=18 && c->code!=19 && c->knob<=1000;
    case RF_SPI_CMD_PITCH: return c->pitch>=0 && c->pitch<=180;
    case RF_SPI_CMD_VISION: return c->power<=100 && c->steering>=-100 &&
                                   c->steering<=100;
    default: return 0;
    }
}
#endif
