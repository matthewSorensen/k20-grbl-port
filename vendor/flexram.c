#include <stdint.h>
#include "mk20dx128.h"
// Per page 552 of the K20 manual, this results in 1024 bytes of NVM,
// with ~32k word aligned write.
#define NVM_SIZE 0x34 
// Oddly, I can't find this definition in the manual?
#define FlexBase ((uint32_t*) 0x14000000)

static void flexram_wait(void){
  uint32_t count = 2000;
  while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY))
    if(count-->0) break;
}

void initialize_flexram(void){
  uint16_t flash_cmd[] = {0xf06f, 0x037f, 0x7003, 0x7803,0xf013, 0x0f80, 0xd0fb, 0x4770};
  uint8_t status;


  SIM_SCGC6 |= (1<<18);// Also enable the clock to the crc

  if (FTFL_FCNFG & FTFL_FCNFG_RAMRDY) {
    // FlexRAM is configured as traditional RAM
    // We need to reconfigure for EEPROM usage
    FTFL_FCCOB0 = 0x80; // PGMPART = Program Partition Command
    FTFL_FCCOB4 = NVM_SIZE;
    FTFL_FCCOB5 = 0x03; // 0K for Dataflash, 32K for EEPROM backup
    __disable_irq();
    // flash_cmd() must execute from RAM.  Luckily the C syntax is simple...
    (*((void (*)(volatile uint8_t *))((uint32_t)flash_cmd | 1)))(&FTFL_FSTAT);
    __enable_irq();
    status = FTFL_FSTAT;
    if (status & 0x70) {
      FTFL_FSTAT = (status & 0x70);
      return; // error
    }
  }
 
  flexram_wait();
}

uint8_t update_checksum(uint8_t checksum, uint32_t word){
  int i;
  for(i = 0; i < 4; i++){
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += word & 0xFF;
    word = word >> 8;
  }
  return checksum;
}

uint32_t write_with_checksum(uint32_t* src, uint32_t dest_offset, uint32_t size){
  uint32_t* target = &FlexBase[dest_offset];
  uint32_t value;
  uint8_t checksum = 0;
  
  for(; size>0; size--){
    value = *src++;
    checksum = update_checksum(checksum, value);
    if(*target != value){
      *target = value;
      flexram_wait();
    }
    target++;
  }
  if(*target != checksum){
    *target = checksum;
    flexram_wait();
  }
  return checksum;
}

uint32_t read_with_checksum(uint32_t* dest, uint32_t src_offset, uint32_t size){
  uint32_t* src = &FlexBase[src_offset];
  uint32_t value;
  uint8_t checksum = 0;
  for(; size > 0; size--){
    value = *src++;
    checksum = update_checksum(checksum, value);
    *dest++ = value;
  }
  return checksum ^ *src;
}

void word_write(uint32_t addr, uint32_t value){
  uint32_t* target = &FlexBase[addr];
  if(*target != value){
    *target = value;
    flexram_wait();
  }
  
}
uint32_t word_read(uint32_t addr){
  return FlexBase[addr];
}
