#include "spi_link_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
int main(void) {
    assert(rf_spi_crc32((const uint8_t*)"123456789",9)==UINT32_C(0xcbf43926));
    uint8_t frame[RF_SPI_FRAME_BYTES], corrupted[RF_SPI_FRAME_BYTES];
    rf_spi_begin(frame,RF_SPI_REQUEST,UINT32_C(0x12345678),UINT32_C(0x01020304),7,9000);
    rf_spi_command_t command={0};
    command.id=42;command.type=RF_SPI_CMD_MOTION;command.action=1;
    command.mode=3;command.power=55;command.strength=40;
    rf_spi_command_write(frame,&command);rf_spi_seal(frame);
    assert(frame[8]==0x78 && frame[11]==0x12 && frame[12]==4 && frame[15]==1);
    assert(frame[24]==42 && frame[28]==1 && frame[29]==1 && frame[30]==3);
    assert(rf_spi_valid(frame,sizeof(frame),RF_SPI_REQUEST));
    assert(!rf_spi_valid(frame,sizeof(frame)-1,RF_SPI_REQUEST));
    assert(!rf_spi_valid(frame,sizeof(frame),RF_SPI_RESPONSE));
    rf_spi_command_t decoded;rf_spi_command_read(frame,&decoded);
    assert(decoded.id==42 && decoded.mode==3 && decoded.power==55);
    for(unsigned byte=0;byte<RF_SPI_FRAME_BYTES;byte++) {
        memcpy(corrupted,frame,sizeof(frame));corrupted[byte]^=1;
        assert(!rf_spi_valid(corrupted,sizeof(corrupted),RF_SPI_REQUEST));
    }
    memcpy(corrupted,frame,sizeof(frame));corrupted[3]='2';rf_spi_seal(corrupted);
    assert(!rf_spi_valid(corrupted,sizeof(corrupted),RF_SPI_REQUEST));
    memcpy(corrupted,frame,sizeof(frame));rf_spi_put16(corrupted+6,99);rf_spi_seal(corrupted);
    assert(!rf_spi_valid(corrupted,sizeof(corrupted),RF_SPI_REQUEST));
    assert(rf_spi_after(1,UINT32_C(0xffffffff)));
    assert(!rf_spi_after(42,42) && !rf_spi_after(41,42));
    /* A restarted peer can reset its own sequence; fresh host ACK still advances. */
    assert(rf_spi_after(1001,1000) && !rf_spi_after(1,900));
    assert(!rf_spi_after(1000,1000)); /* frozen cached response cannot refresh liveness */
    command.type=RF_SPI_CMD_LORA;command.code=8;
    assert(!rf_spi_command_valid(&command));
    command.code=1;command.knob=1001;assert(!rf_spi_command_valid(&command));
    command.type=RF_SPI_CMD_VISION;command.power=100;command.steering=-100;
    assert(rf_spi_command_valid(&command));
    command.steering=-101;assert(!rf_spi_command_valid(&command));
    command.type=4;
    assert(!rf_spi_command_valid(&command)); /* removed demonstration command */
    rf_spi_begin(frame,RF_SPI_RESPONSE,1,2,3,4);
    frame[5]=RF_SPI_READY|RF_SPI_WIFI_CONTROL_ALLOWED;
    frame[61]=1;rf_spi_put16(frame+62,100);rf_spi_put16(frame+78,7400);rf_spi_seal(frame);
    assert(rf_spi_valid(frame,sizeof(frame),RF_SPI_RESPONSE));
    assert(frame[61] && rf_spi_u16(frame+62)==100 && rf_spi_u16(frame+78)==7400);
    frame[78]^=1;assert(!rf_spi_valid(frame,sizeof(frame),RF_SPI_RESPONSE));
    printf("PASS: CRC vector, 128 corruption cases, endian frame, version/length/type rejection, wrap/replay/reset ACK, command bounds.\n");
    return 0;
}
