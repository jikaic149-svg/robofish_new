#include <cassert>
#include <cstdio>
#include <cstring>
#include "main.h"
#include "service/lora/LoRaService.h"
TIM_HandleTypeDef htim1,htim2,htim3,htim4,htim5;
UART_HandleTypeDef huart1;
struct Sink {unsigned applied=0;robofish::lora::Command last{};};
bool submit(const robofish::lora::Command &c,void *context) {
  auto &sink=*static_cast<Sink*>(context);++sink.applied;sink.last=c;return true;
}
void receive(const char *line) {while(*line){*rxByte=uint8_t(*line++);robofish::bsp::onUartRxComplete();}}
int main() {
  using namespace robofish::lora;
  huart1.Instance=USART1;
  Sink sink;LoRaService service(submit,&sink);assert(service.begin());
  uint32_t now=0;
  auto send=[&](unsigned source,unsigned sequence,unsigned code) {
    char line[64];std::snprintf(line,sizeof(line),"100,1,%u,%u,%u,1000\n",source,sequence,code);
    receive(line);service.poll(++now);
  };
  send(0,100,2);send(0,101,17);send(0,99,2);
  assert(sink.applied==2 && sink.last.code==17 && service.status().stale==1);
  send(0,101,17);assert(sink.applied==2 && service.status().duplicates==1);
  send(0,101,2);assert(sink.applied==2); // Conflicting payload for the same key.
  for(unsigned seq=102;seq<108;++seq)send(0,seq,17);
  unsigned applied=sink.applied;send(0,100,2);assert(sink.applied==applied);
  send(1,100,2);assert(sink.applied==applied+1 && service.status().lastSource==1);
  send(2,65534,2);send(2,65535,17);send(2,0,2);assert(sink.last.source==2 && sink.last.sequence==0);
  applied=sink.applied;send(2,65535,2);send(2,32768,2);assert(sink.applied==applied);
  send(3,10,17);applied=sink.applied;send(4,1,2);assert(sink.applied==applied); // No history eviction.
  receive("100,1,0,108,");service.poll(++now);
  robofish::bsp::onUartError();service.poll(++now);
  receive("2,1000\n");service.poll(++now);assert(sink.applied==applied);
  send(0,109,17);assert(sink.applied==applied+1);
  service.stop();service.poll(++now);
  // Verified timing in this simulation only; production ACK remains disabled.
  robofish::config::radio::Timing timing{true,true,true,64,100,10,100,20,30};
  Sink ackSink;LoRaService ackService(submit,&ackSink,timing);assert(ackService.begin());
  receive("100,1,7,1,5,1000\n");ackService.poll(0);ackService.poll(10);ackService.poll(11);
  const char expected[]="910,7,1,1,7,5,1\n";
  assert(txLength==sizeof(expected)-1 && std::memcmp(txBytes,expected,txLength)==0);
  huart1.gState=HAL_UART_STATE_READY;ackService.poll(12);ackService.poll(42);
  receive("100,1,7,1,5,1000\n");ackService.poll(100);ackService.poll(110);ackService.poll(111);
  assert(ackSink.applied==1 && ackService.status().duplicates==1);
  assert(txLength==sizeof(expected)-1 && std::memcmp(txBytes,expected,txLength)==0);
  Command c;assert(!parseLine("100,1,65536,1,2",c));assert(!parseLine("100,1,-1,1,2",c));
  puts("PASS: source/sequence ordering, stop barrier, duplicates/ACK, cache eviction, wrap/half-range, UART error resync");
}
