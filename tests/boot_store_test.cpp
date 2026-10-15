#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "boot/BootStore.h"

alignas(8) static uint32_t flashWords[1024*1024/4];
static int programBudget=-1, eraseBudget=-1;
static uint8_t *memory(uint32_t address) {
  assert(address>=RF_BOOT_BASE && address<RF_BOOT_BASE+sizeof(flashWords));
  return reinterpret_cast<uint8_t *>(flashWords)+(address-RF_BOOT_BASE);
}
extern "C" const uint8_t *rf_host_flash(uint32_t address) {return memory(address);}
extern "C" int rf_host_program(uint32_t address,uint32_t value) {
  assert(address>=RF_RECORD0 && address<RF_RECORD1+RF_RECORD_BYTES && !(address&3));
  if(programBudget==0)return 0;
  if(programBudget>0)--programBudget;
  uint32_t old;std::memcpy(&old,memory(address),4);
  assert((old&value)==value);std::memcpy(memory(address),&value,4);return 1;
}
extern "C" int rf_host_erase(uint32_t page) {
  assert(page==RF_RECORD0 || page==RF_RECORD1);
  unsigned words=eraseBudget<0 ? RF_RECORD_BYTES/4 : static_cast<unsigned>(eraseBudget);
  assert(words<=RF_RECORD_BYTES/4);
  std::memset(memory(page),0xFF,words*4);
  return eraseBudget<0;
}
static void reset() {std::memset(flashWords,0xFF,sizeof(flashWords));programBudget=eraseBudget=-1;}
static rf_boot_record_t factory() {
  rf_boot_record_t r{};
  r.magic=RF_BOOT_MAGIC;r.format=RF_BOOT_FORMAT;r.sequence=1;
  r.confirmed=0;r.candidate=RF_BOOT_NONE;r.state=RF_BOOT_STABLE;r.confirmed_mask=1;
  for(unsigned slot=0;slot<2;++slot) {
    auto p=memory(rf_app_base(slot));std::memset(p,0x42,516);std::memset(p,0,RF_VECTOR_BYTES);
    uint32_t sp=RF_FAULT_RAM,pc=rf_app_base(slot)+401;
    std::memcpy(p,&sp,4);std::memcpy(p+4,&pc,4);
    r.image[slot].size=513;r.image[slot].crc=rf_crc_bytes(p,513);
  }
  r.crc=rf_crc_bytes(&r,offsetof(rf_boot_record_t,crc));r.commit=RF_BOOT_COMMIT;
  return r;
}
int main(int argc,char **argv) {
  const uint8_t zero[4]={0,0,0,0},ff[4]={255,255,255,255};
  assert(rf_crc_bytes(zero,4)==0xC704DD7BU);assert(rf_crc_bytes(ff,4)==0);
  for(int cut=0;cut<=30;++cut) {
    reset();auto r=factory();assert(rf_boot_append(&r));
    auto next=r;next.sequence=2;next.candidate=1;next.state=RF_BOOT_PENDING;
    programBudget=cut;bool committed=rf_boot_append(&next);
    rf_boot_record_t got;assert(rf_boot_latest(&got,nullptr));
    assert(got.sequence==(committed?2U:1U));
    assert(got.confirmed==0);
  }
  // Power loss during reclaim only damages the older page, never the latest record.
  for(int cut : {0,1,127,1024,4095,4096}) {
    reset();auto r=factory();
    for(unsigned page=0;page<2;++page) for(unsigned row=0;row<128;++row) {
      r.sequence=(page==0?1000U:1U)+row;
      r.crc=rf_crc_bytes(&r,offsetof(rf_boot_record_t,crc));
      std::memcpy(memory((page?RF_RECORD1:RF_RECORD0)+row*RF_RECORD_STRIDE),&r,sizeof(r));
    }
    rf_boot_record_t latest;assert(rf_boot_latest(&latest,nullptr)&&latest.sequence==1127);
    auto next=latest;next.sequence=1128;eraseBudget=cut;
    assert(!rf_boot_append(&next));assert(rf_boot_latest(&latest,nullptr)&&latest.sequence==1127);
  }
  reset();auto r=factory();assert(rf_boot_append(&r));
  assert(rf_boot_image_valid(0,&r.image[0]) && rf_boot_image_valid(1,&r.image[1]));
  memory(RF_APP_B)[512]^=1; // Legal header + corrupt final payload must be rejected.
  assert(!rf_boot_image_valid(1,&r.image[1]));
  auto pending=r;pending.candidate=1;pending.state=RF_BOOT_PENDING;
  assert(rf_boot_select(&pending,1)==0 && pending.state==RF_BOOT_STABLE);
  pending=r;pending.candidate=1;pending.state=RF_BOOT_PENDING;
  auto illegal=pending;illegal.confirmed_mask=3;
  illegal.crc=rf_crc_bytes(&illegal,offsetof(rf_boot_record_t,crc));
  assert(!rf_boot_record_valid(&illegal)); // A new candidate cannot claim prior confirmation.
  assert(rf_boot_select(&pending,3)==1 && pending.state==RF_BOOT_TRYING && pending.attempts==1);
  assert(rf_boot_append(&pending));
  rf_boot_record_t after;assert(rf_boot_latest(&after,nullptr));
  assert(rf_boot_select(&after,3)==0 && after.state==RF_BOOT_STABLE);
  assert(rf_boot_append(&after));
  assert(rf_boot_select(&after,0)==RF_BOOT_NONE);
  auto fallback=r;fallback.confirmed_mask=3;
  assert(rf_boot_select(&fallback,2)==1 && fallback.confirmed==1);
  reset();r=factory();r.sequence=0xFFFFFFFFU;assert(rf_boot_append(&r));
  r.sequence=0;assert(rf_boot_append(&r));assert(rf_boot_latest(&after,nullptr)&&after.sequence==0);
  assert(!rf_app_vectors_valid(RF_APP_A,513,RF_FAULT_RAM,RF_APP_A+3));
  assert(!rf_app_vectors_valid(RF_APP_A,513,RF_FAULT_RAM+8,RF_APP_A+401));
  assert(!rf_app_vectors_valid(RF_APP_A,RF_APP_MAX_BYTES+1,RF_FAULT_RAM,RF_APP_A+401));
  for(int i=1;i<argc;++i) {
    reset();FILE *file=std::fopen(argv[i],"rb");assert(file);
    const auto count=std::fread(flashWords,1,sizeof(flashWords),file);
    assert(count>0 && std::fgetc(file)==EOF);std::fclose(file);
    assert(rf_boot_latest(&after,nullptr));
    assert(after.state==RF_BOOT_STABLE && after.confirmed_mask==3);
    assert(rf_boot_image_valid(0,&after.image[0]) && rf_boot_image_valid(1,&after.image[1]));
  }
  puts("PASS: native CRC vectors/tail, every record-write cut, page-reclaim cuts, corrupt image tail, persisted trial/rollback and generation wrap");
}
