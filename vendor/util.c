#include "util.h"
extern volatile uint32_t systick_millis_count;

void delay(uint32_t ms){
  uint32_t start = systick_millis_count;
  uint32_t end = start + ms;

  if(end < start) // If our time interval causes an overflow, wait for the count to overflow and proceed as normal
    while(systick_millis_count != 0);
  while(systick_millis_count < end);
}

