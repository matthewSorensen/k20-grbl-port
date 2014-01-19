#include <mk20dx128.h>
#include <pin_config.h>                  
#include <util.h>

#include "pin_map.h"

#define TIE 2 // Timer interrupt enable
#define TEN 1 // Timer enable

void st_init(void);
static volatile uint32_t out_bits;

#define INVERT_MASK (DIR_Y_BIT | STEP_X_BIT)

int main(void){
  st_init();
  out_bits = STEP_MASK | DIRECTION_MASK;
  PIT_TCTRL0 |= TIE;
  STEPPER_PORT(SOR) = DIR_Y_BIT; // Make y bit is in an a state that tests that we do indeed reset it.
  while(1);
}

void st_init()
{
  // Configure directions of interface pins
  STEPPER_DDR = STEP_MASK | DIRECTION_MASK | STEPPER_DISABLE_BIT;
  STEP_X_CTRL = STANDARD_OUTPUT;
  STEP_Y_CTRL = STANDARD_OUTPUT;
  STEP_Z_CTRL = STANDARD_OUTPUT;
  DIR_X_CTRL = STANDARD_OUTPUT;
  DIR_Y_CTRL = STANDARD_OUTPUT;
  DIR_Z_CTRL = STANDARD_OUTPUT;
  STEPPER_DISABLE_CTRL = STANDARD_OUTPUT;
  // Configure PIT module
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  // Configure PIT 0 - main interrupt timer
  PIT_LDVAL0 = 4800;
  PIT_TCTRL0 = TEN; // Keep it running at a reasonable speed, but not interrupting 
  // Configure PIT 1 - reset timer
  PIT_TCTRL1 = TIE;
  PIT_LDVAL1 = 480; 
  // Start in the idle state, but first wake up to check for keep steppers enabled option.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
}

void pit0_isr(void){
  PIT_TFLG0 = 1;
 
  STEPPER_PORT(SOR) = (~INVERT_MASK) & out_bits;
  STEPPER_PORT(COR) = INVERT_MASK & out_bits;

  PIT_LDVAL1 = 480; // 10 us
  PIT_TCTRL1 |= TEN;
}

void pit1_isr(void){
  PIT_TFLG1 = 1;
  PIT_TCTRL1 &= ~TEN;

  STEPPER_PORT(TOR) = out_bits & STEP_MASK;
}
