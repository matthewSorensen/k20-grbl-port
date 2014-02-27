#ifndef flexram_h
#define flexram_h

void initialize_flexram(void);
uint32_t write_with_checksum(uint32_t*, uint32_t, uint32_t);
uint32_t read_with_checksum(uint32_t*,uint32_t,uint32_t);

void     word_write(uint32_t, uint32_t);
uint32_t word_read(uint32_t);


#endif
