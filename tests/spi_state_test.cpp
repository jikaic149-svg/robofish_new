#include <cassert>
#include <cstdio>
#include "../stm32f405/stm32f405/bsp/SpiLink.cpp"
void csRise() {mockCsHigh=true;exti.pending|=GPIO_PIN_12;EXTI15_10_IRQHandler();}
void clocks() {mockCsHigh=false;dmaRx.NDTR=dmaTx.NDTR=0;}
void rxDone() {receiveDma.complete=true;DMA1_Stream3_IRQHandler();}
void txDone() {transmitDma.complete=true;DMA1_Stream4_IRQHandler();}
void drain() {uint8_t packet[RF_SPI_FRAME_BYTES];while(xQueueReceive(receiveQueue,packet,0)==pdTRUE){}}
int main() {
  MotionController motion;assert(robofish::bsp::startSpiLink(motion));assert(mockStarts==1);
  uint8_t snapshot[RF_SPI_FRAME_BYTES];std::memcpy(snapshot,transmitBytes,sizeof(snapshot));
  publishedBytes[50]^=0xff;assert(std::memcmp(snapshot,transmitBytes,sizeof(snapshot))==0);
  clocks();rxDone();assert(mockStarts==1 && receiveQueue->count==0);
  csRise();assert(mockStarts==1);txDone();assert(mockStarts==2 && receiveQueue->count==1);drain();
  // TX first; both DMA handlers complete while CS is still held low.
  clocks();txDone();rxDone();assert(mockStarts==2 && receiveQueue->count==0);
  csRise();assert(mockStarts==3 && receiveQueue->count==1);drain();
  // CS first, before either DMA completion IRQ.
  clocks();csRise();assert(mockStarts==3);txDone();assert(mockStarts==3);rxDone();assert(mockStarts==4);drain();
  // No clocks, no buffer overwrite or false completion.
  mockCsHigh=false;csRise();assert(mockStarts==4 && receiveQueue->count==0);
  // Short frame: stop requests, wait for stream EN acknowledgement without spinning.
  mockCsHigh=false;dmaRx.NDTR=64;mockDelayedDisable=true;csRise();
  assert(phase==TransferPhase::Recovering && mockStarts==4 && receiveQueue->count==0);
  (void)progressTransfer(false);assert(mockStarts==4);
  mockDelayedDisable=false;dmaRx.CR=dmaTx.CR=0;(void)progressTransfer(false);
  assert(mockStarts==5 && phase==TransferPhase::Active && mockResets==1);
  // A startup failure after RX DMA has started must be fully unwound.
  clocks();txDone();mockFailStart=true;rxDone();csRise();
  assert(mockStarts==6 && phase==TransferPhase::Recovering);
  (void)progressTransfer(false);assert(mockStarts==7 && phase==TransferPhase::Active);drain();
  // Extra clocks/OVR invalidate the frame before it reaches the protocol task.
  clocks();txDone();rxDone();spi2.SR|=SPI_FLAG_OVR;csRise();
  assert(phase==TransferPhase::Recovering && receiveQueue->count==0);
  (void)progressTransfer(false);assert(mockStarts==8);
  HAL_SPI_ErrorCallback(&spi);assert(phase==TransferPhase::Recovering);
  (void)progressTransfer(false);assert(mockStarts==9);
  for(unsigned i=0;i<5;++i){clocks();txDone();rxDone();csRise();}
  assert(receiveQueue->count==4 && dropped.load()==1 && mockStarts==14);
  assert(transferErrors.load()==4);
  puts("PASS: real SPI BSP with RX/TX/CS permutations, immutable TX, short/long frames, partial start, delayed DMA disable, error recovery and queue overflow");
}
