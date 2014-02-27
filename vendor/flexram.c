#include <stdint.h>
#include "mk20dx128.h"
// Per page 552 of the K20 manual, this results in 1024 bytes of NVM,
// with ~32k word aligned write.
#define NVM_SIZE 0x34 
// Oddly, I can't find this definition in the manual?
#define FlexBase ((uint32_t*) 0x14000000)

#define CRC_32C 0x8F6E37A0
#define WAS (1<<25)
#define TCRC (1<<24)

static void flexram_wait(void){
  uint32_t count = 2000;
  while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY))
    if(count-->0) break;
}

static void configure_crc(void){
  CRC_GPOLY = CRC_32C;
  CRC_CTRL = 0 | WAS | TCRC; // 32 bit wide (TCRC), and in mode to write seed
  CRC_CRC = 0; // Write seed
  CRC_CTRL &= ~WAS;
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

uint32_t write_with_checksum(uint32_t* src, uint32_t dest_offset, uint32_t size){
  uint32_t* target = &FlexBase[dest_offset];
  uint32_t value;
  configure_crc();
  for(; size>0; size--){
    value = *src++;
    CRC_CRC = value;
    if(*target != value){
      *target = value;
      flexram_wait();
    }
    target++;
  }
  if(*target != CRC_CRC){
    *target = CRC_CRC;
    flexram_wait();
  }
  return CRC_CRC;
}


uint32_t read_with_checksum(uint32_t* dest, uint32_t src_offset, uint32_t size){
  uint32_t* src = &FlexBase[src_offset];
  uint32_t value;
  configure_crc();
  for(; size > 0; size--){
    value = *src++;
    CRC_CRC = value;
    *dest++ = value;
  }
  return CRC_CRC ^ *src;
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
