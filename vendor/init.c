#include "mk20dx128.h"

#include "usb_dev.h"

#if F_BUS == 48000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 1
#else
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 0
#endif

void init(void){
  // Convince the interrupt controler to let GPIO pins trigger interrupts
  NVIC_ENABLE_IRQ(IRQ_PORTA);
  NVIC_ENABLE_IRQ(IRQ_PORTB);
  NVIC_ENABLE_IRQ(IRQ_PORTC);
  NVIC_ENABLE_IRQ(IRQ_PORTD);
  NVIC_ENABLE_IRQ(IRQ_PORTE);

  // ????   
  FTM0_CNT = 0;
  FTM0_MOD = DEFAULT_FTM_MOD;
  FTM0_C0SC = 0x28; // MSnB:MSnA = 10, ELSnB:ELSnA = 10
  FTM0_C1SC = 0x28;
  FTM0_C2SC = 0x28;
  FTM0_C3SC = 0x28;
  FTM0_C4SC = 0x28;
  FTM0_C5SC = 0x28;
  FTM0_C6SC = 0x28;
  FTM0_C7SC = 0x28;
  FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
  FTM1_CNT = 0;
  FTM1_MOD = DEFAULT_FTM_MOD;
  FTM1_C0SC = 0x28;
  FTM1_C1SC = 0x28;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);

  // This is where we should initialize the analog subsystem
  usb_init();
}
