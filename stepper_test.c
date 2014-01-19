#include <mk20dx128.h>
#include <pin_config.h>                  
//#include <util.h>
#include <math.h>

#include "nuts_bolts.h"
#include "pin_map.h"

#define TIE 2 // Timer interrupt enable
#define TEN 1 // Timer enable

#define ALL ((1<<Y_AXIS)|(1<<Z_AXIS)|(1<<X_AXIS))

#define ACCELERATION_TICKS_PER_SECOND 50L
#define MICROSECONDS_PER_ACCELERATION_TICK  (1000000/ACCELERATION_TICKS_PER_SECOND)


void delay_microseconds(uint32_t usec)
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
		"L_%=delay_microseconds:"		"\n\t"
		"subs   %0, #1"				"\n\t"
		"bne    L_%=delay_microseconds"		"\n"
		: "+r" (n) :
	);
}



void st_init(void);
void limits_init();
static void homing_cycle(uint32_t, int32_t, bool, float); 


static volatile uint32_t out_bits;

#define INVERT_MASK (DIR_Y_BIT)

int main(void){
  st_init();
  limits_init();
  while(1) homing_cycle(ALL,ALL,0,600);

}
void limits_init() 
{
  uint32_t config_reg;
  LIMIT_DDR &= ~(LIMITS_MASK | RESET_BIT | FEED_HOLD_BIT | CYCLE_BIT); // Set as input pins

  #ifndef LIMIT_SWITCHES_ACTIVE_HIGH
  config_reg = PULL_UP | MUX_GPIO;
  #else
  config_reg = PULL_DOWN | MUX_GPIO;
  #endif
  
  LIMIT_X_CTRL = config_reg;
  LIMIT_Y_CTRL = config_reg;
  LIMIT_Z_CTRL = config_reg;
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

static void homing_cycle(uint32_t cycle_mask, int32_t pos_dir, bool invert_pin, float homing_rate) 
{
  #ifdef LIMIT_SWITCHES_ACTIVE_HIGH
    // When in an active-high switch configuration, invert_pin needs to be adjusted.
    invert_pin = !invert_pin;
  #endif

  // Determine governing axes with finest step resolution per distance for the Bresenham
  // algorithm. This solves the issue when homing multiple axes that have different 
  // resolutions without exceeding system acceleration setting. It doesn't have to be
  // perfect since homing locates machine zero, but should create for a more consistent 
  // and speedy homing routine.
  // NOTE: For each axes enabled, the following calculations assume they physically move 
  // an equal distance over each time step until they hit a limit switch, aka dogleg.
  uint32_t steps[3];
  uint8_t dist = 0;
  clear_vector(steps);
  if (cycle_mask & (1<<X_AXIS)) { 
    dist++;
    steps[X_AXIS] = lround(250); 
  }
  if (cycle_mask & (1<<Y_AXIS)) { 
    dist++;
    steps[Y_AXIS] = lround(250); 
  }
  if (cycle_mask & (1<<Z_AXIS)) {
    dist++;
    steps[Z_AXIS] = lround(250);
  }
  uint32_t step_event_count = max(steps[X_AXIS], max(steps[Y_AXIS], steps[Z_AXIS]));  
  
  // To ensure global acceleration is not exceeded, reduce the governing axes nominal rate
  // by adjusting the actual axes distance traveled per step. This is the same procedure
  // used in the main planner to account for distance traveled when moving multiple axes.
  // NOTE: When axis acceleration independence is installed, this will be updated to move
  // all axes at their maximum acceleration and rate.
  float ds = step_event_count/sqrt(dist);

  // Compute the adjusted step rate change with each acceleration tick. (in step/min/acceleration_tick)
  uint32_t delta_rate = ceil( ds*10.0*60*60/(60*ACCELERATION_TICKS_PER_SECOND));
  
  #ifdef HOMING_RATE_ADJUST
    // Adjust homing rate so a multiple axes moves all at the homing rate independently.
    homing_rate *= sqrt(dist); // Eq. only works if axes values are 1 or 0.
  #endif
  
  // Nominal and initial time increment per step. Nominal should always be greater then 3
  // usec, since they are based on the same parameters as the main stepper routine. Initial
  // is based on the MINIMUM_STEPS_PER_MINUTE config. Since homing feed can be very slow,
  // disable acceleration when rates are below MINIMUM_STEPS_PER_MINUTE.
  uint32_t dt_min = lround(1000000*60/(ds*homing_rate)); // Cruising (usec/step)
  uint32_t dt = 1000000*60/MINIMUM_STEPS_PER_MINUTE; // Initial (usec/step)
  if (dt > dt_min) { dt = dt_min; } // Disable acceleration for very slow rates.
      
  // Set default out_bits. 
  uint32_t out_bits0 = INVERT_MASK;
  out_bits0 ^= (0 & DIRECTION_MASK); // Apply homing direction settings
  if (!pos_dir) { out_bits0 ^= DIRECTION_MASK; }   // Invert bits, if negative dir.
  
  // Initialize stepping variables
  int32_t counter_x = -(step_event_count >> 1); // Bresenham counters
  int32_t counter_y = counter_x;
  int32_t counter_z = counter_x;
  uint32_t step_delay = 480;  // Step delay after pulse
  uint32_t step_rate = 0;  // Tracks step rate. Initialized from 0 rate. (in step/min)
  uint32_t trap_counter = MICROSECONDS_PER_ACCELERATION_TICK/2; // Acceleration trapezoid counter
  uint32_t out_bits;
  uint32_t limit_state;
  for(;;) {
  
    // Reset out bits. Both direction and step pins appropriately inverted and set.
    out_bits = out_bits0;
    
    // Get limit pin state.
    limit_state = LIMIT_PORT(DIR);
    if (invert_pin) { limit_state ^= LIMITS_MASK; } // If leaving switch, invert to move.
    
    // Set step pins by Bresenham line algorithm. If limit switch reached, disable and
    // flag for completion.
    if (cycle_mask & (1<<X_AXIS)) {
      counter_x += steps[X_AXIS];
      if (counter_x > 0) {
        if (limit_state & LIMIT_X_BIT) { out_bits ^= STEP_X_BIT; }
        else { cycle_mask &= ~(1<<X_AXIS); }
        counter_x -= step_event_count;
      }
    }
    if (cycle_mask & (1<<Y_AXIS)) {
      counter_y += steps[Y_AXIS];
      if (counter_y > 0) {
        if (limit_state & LIMIT_Y_BIT) { out_bits ^= STEP_Y_BIT; }
        else { cycle_mask &= ~(1<<Y_AXIS); }
        counter_y -= step_event_count;
      }
    }
    if (cycle_mask & (1<<Z_AXIS)) {
      counter_z += steps[Z_AXIS];
      if (counter_z > 0) {
        if (limit_state & LIMIT_Z_BIT) { out_bits ^= STEP_Z_BIT; }
        else { cycle_mask &= ~(1<<Z_AXIS); }
        counter_z -= step_event_count;
      }
    }        
    
    // Perform step.
    // Set the direction pins a couple of nanoseconds before we step the steppers
    //    STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
    //delay_micoseconds(step_delay);
    //STEPPER_PORT(SOR) = out_bits;
    //delay_microseconds(500); //settings.pulse_microseconds);
    //STEPPER_PORT(COR) = STEP_MASK;

    while(1){
      STEPPER_PORT(SOR) = (~INVERT_MASK) & out_bits;
      STEPPER_PORT(COR) = INVERT_MASK & out_bits;
      delay_microseconds(480);
      STEPPER_PORT(TOR) = out_bits & STEP_MASK;
      delay_microseconds(10000);
    }
  
  
  // Track and set the next step delay, if required. This routine uses another Bresenham
  // line algorithm to follow the constant acceleration line in the velocity and time 
  // domain. This is a lite version of the same routine used in the main stepper program.
  if (dt > dt_min) { // Unless cruising, check for time update.
    trap_counter += dt; // Track time passed since last update.
    if (trap_counter > MICROSECONDS_PER_ACCELERATION_TICK) {
      trap_counter -= MICROSECONDS_PER_ACCELERATION_TICK;
      step_rate += delta_rate; // Increment velocity
      dt = (1000000*60)/step_rate; // Compute new time increment
      if (dt < dt_min) {dt = dt_min;}  // If target rate reached, cruise.
      step_delay = dt-480;
    }
  }
  }
}
