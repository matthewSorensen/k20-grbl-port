#ifndef util_h
#define util_h

#include <stdint.h>

void delay(uint32_t);
static inline void delay_microseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delay_microseconds(uint32_t usec)
{
#if F_CPU == 96000000
	uint32_t n = usec << 5;
#elif F_CPU == 48000000
	uint32_t n = usec << 4;
#elif F_CPU == 24000000
	uint32_t n = usec << 3;
#endif
	if (usec == 0) return;
	asm volatile(
		"L_%=_delay_microseconds:"		"\n\t"
		"subs   %0, #1"				"\n\t"
		"bne    L_%=_delay_microseconds"		"\n"
		: "+r" (n) :
	);
}

#endif
