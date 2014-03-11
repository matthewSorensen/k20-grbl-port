/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "stepper.h"
#include "config.h"
#include "settings.h"
#include "planner.h"

#include <mk20dx128.h>
#include <pin_config.h>                  
#include <util.h>

// Some useful constants
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  uint32_t event_count;
  uint32_t step_events_completed;  // The number of step events left in current motion

  // Used by the trapezoid generator
  uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
  uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
  uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator
  uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.
} stepper_t;

static stepper_t st;
static block_t *current_block;  // A pointer to the block currently being traced

// Bits in the PIT register:
#define TIE 2 // Timer interrupt enable
#define TEN 1 // Timer enable
enum pulse_status {PULSE_SET, PULSE_RESET};

typedef struct {
  volatile uint32_t active_bits;
  uint32_t pulse_length;
  volatile enum pulse_status step_interrupt_status;
} pulse_state;

static pulse_state pit1_state;

// Used by the stepper driver interrupt
volatile uint32_t out_bits;

volatile uint32_t busy;   // True when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate following the midpoint rule
//  by the trapezoid generator, which is called ACCELERATION_TICKS_PER_SECOND times per second.

static void set_step_events_per_minute(uint32_t steps_per_minute);

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up() 
{
  // Before we enable the stepper motors, make sure that the step bits are set to not trigger a pulse
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~STEP_MASK) | (STEP_MASK & settings.invert_mask);

  // Enable steppers by resetting the stepper disable port
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { 
    STEPPER_DISABLE_PORT(SOR) = STEPPER_DISABLE_BIT; 
  } else { 
    STEPPER_DISABLE_PORT(COR) = STEPPER_DISABLE_BIT;
  }
  if (sys.state == STATE_CYCLE) {
    // Initialize stepper output bits
    out_bits = (0) ^ (settings.invert_mask); 
    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    pit1_state.pulse_length = settings.pulse_microseconds*TICKS_PER_MICROSECOND;
    // Enable stepper driver interrupt
    PIT_TCTRL0 |= TIE;
  }
}

// Stepper shutdown
void st_go_idle() 
{
  // Disable stepper driver interrupt
  PIT_TCTRL0 &= ~TIE; 
  // Disable steppers only upon system alarm activated or by user setting to not be kept enabled.
  if ((settings.stepper_idle_lock_time != 0xff) || bit_istrue(sys.execute,EXEC_ALARM)) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay_microseconds(1000 * settings.stepper_idle_lock_time);
    if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { 
      STEPPER_DISABLE_PORT(COR) = STEPPER_DISABLE_BIT;
    }else{
      STEPPER_DISABLE_PORT(SOR) = STEPPER_DISABLE_BIT; 
    }   
  }
}

// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that 
// step_events occur significantly more often than the acceleration velocity iterations.
inline static uint32_t iterate_trapezoid_cycle_counter() 
{
  st.trapezoid_tick_cycle_counter += st.cycles_per_step_event;  
  if(st.trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    st.trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } else {
    return(false);
  }
}          


					  
// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse. 
// The bresenham line tracer algorithm controls all three stepper outputs simultaneously with these two interrupts.
void pit0_isr(void) {
  PIT_TFLG0 = 1;

  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  // Set the direction bits. Todo: only do this at the start of a block?
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  trigger_pulse(out_bits & STEP_MASK);

  busy = true;
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer? If so, initialize next motion.
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      if (sys.state == STATE_CYCLE) {
        // During feed hold, do not update rate and trap counter. Keep decelerating.
        st.trapezoid_adjusted_rate = current_block->initial_rate;
        set_step_events_per_minute(st.trapezoid_adjusted_rate); // Initialize cycles_per_step_event
        st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
      }
      st.min_safe_rate = current_block->rate_delta + (current_block->rate_delta >> 1); // 1.5 x rate_delta
      st.counter_x = -(current_block->step_event_count >> 1);
      st.counter_y = st.counter_x;
      st.counter_z = st.counter_x;
      st.event_count = current_block->step_event_count;
      st.step_events_completed = 0;     
    } else {
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
    }    
  } 

  if (current_block != NULL) {
    // Execute step displacement profile by bresenham line algorithm
    out_bits = current_block->direction_bits;
    st.counter_x += current_block->steps_x;
    if (st.counter_x > 0) {
      out_bits |= STEP_X_BIT;
      st.counter_x -= st.event_count;
      if (out_bits & DIR_X_BIT) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y += current_block->steps_y;
    if (st.counter_y > 0) {
      out_bits |= STEP_Y_BIT;
      st.counter_y -= st.event_count;
      if (out_bits & DIR_Y_BIT) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z += current_block->steps_z;
    if (st.counter_z > 0) {
      out_bits |= STEP_Z_BIT;
      st.counter_z -= st.event_count;
      if (out_bits & DIR_Z_BIT) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }
    
    st.step_events_completed++; // Iterate step events

    // While in block steps, check for de/ac-celeration events and execute them accordingly.
    if (st.step_events_completed < current_block->step_event_count) {
      if (sys.state == STATE_HOLD) {
        // Check for and execute feed hold by enforcing a steady deceleration from the moment of 
        // execution. The rate of deceleration is limited by rate_delta and will never decelerate
        // faster or slower than in normal operation. If the distance required for the feed hold 
        // deceleration spans more than one block, the initial rate of the following blocks are not
        // updated and deceleration is continued according to their corresponding rate_delta.
        // NOTE: The trapezoid tick cycle counter is not updated intentionally. This ensures that 
        // the deceleration is smooth regardless of where the feed hold is initiated and if the
        // deceleration distance spans multiple blocks.
        if ( iterate_trapezoid_cycle_counter() ) {                    
          // If deceleration complete, set system flags and shutdown steppers.
          if (st.trapezoid_adjusted_rate <= current_block->rate_delta) {
            // Just go idle. Do not NULL current block. The bresenham algorithm variables must
            // remain intact to ensure the stepper path is exactly the same. Feed hold is still
            // active and is released after the buffer has been reinitialized.
            st_go_idle();
            bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program that feed hold is complete.
          } else {
            st.trapezoid_adjusted_rate -= current_block->rate_delta;
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }      
        }
        
      } else {
        // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
        // executed and terminated at exactly the right time. This helps prevent over/under-shooting
        // the target position and speed. 
        // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
        // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
        // round-off errors can effect this, if set too high. This is important to note if a user has 
        // very high acceleration and/or feedrate requirements for their machine.
        if (st.step_events_completed < current_block->accelerate_until) {
          // Iterate cycle counter and check if speeds need to be increased.
          if ( iterate_trapezoid_cycle_counter() ) {
            st.trapezoid_adjusted_rate += current_block->rate_delta;
            if (st.trapezoid_adjusted_rate >= current_block->nominal_rate) {
              // Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
              st.trapezoid_adjusted_rate = current_block->nominal_rate;
            }
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }
        } else if (st.step_events_completed >= current_block->decelerate_after) {
          // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
          // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
          // an accurate approximation of the deceleration curve. For triangle profiles, down count
          // from current cycle counter to ensure exact deceleration curve.
          if (st.step_events_completed == current_block-> decelerate_after) {
            if (st.trapezoid_adjusted_rate == current_block->nominal_rate) {
              st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Trapezoid profile
            } else {  
              st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK-st.trapezoid_tick_cycle_counter; // Triangle profile
            }
          } else {
            // Iterate cycle counter and check if speeds need to be reduced.
            if ( iterate_trapezoid_cycle_counter() ) {  
              // NOTE: We will only do a full speed reduction if the result is more than the minimum safe 
              // rate, initialized in trapezoid reset as 1.5 x rate_delta. Otherwise, reduce the speed by
              // half increments until finished. The half increments are guaranteed not to exceed the 
              // CNC acceleration limits, because they will never be greater than rate_delta. This catches
              // small errors that might leave steps hanging after the last trapezoid tick or a very slow
              // step rate at the end of a full stop deceleration in certain situations. The half rate 
              // reductions should only be called once or twice per block and create a nice smooth 
              // end deceleration.
              if (st.trapezoid_adjusted_rate > st.min_safe_rate) {
                st.trapezoid_adjusted_rate -= current_block->rate_delta;
              } else {
                st.trapezoid_adjusted_rate >>= 1; // Bit shift divide by 2
              }
              if (st.trapezoid_adjusted_rate < current_block->final_rate) {
                // Reached final rate a little early. Cruise to end of block at final rate.
                st.trapezoid_adjusted_rate = current_block->final_rate;
              }
              set_step_events_per_minute(st.trapezoid_adjusted_rate);
            }
          }
        } else {
          // No accelerations. Make sure we cruise exactly at the nominal rate.
          if (st.trapezoid_adjusted_rate != current_block->nominal_rate) {
            st.trapezoid_adjusted_rate = current_block->nominal_rate;
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }
        }
      }            
    } else {   
      // If current block is finished, reset pointer 
      current_block = NULL;
      plan_discard_current_block();
    }
  }
  out_bits ^= settings.invert_mask & DIRECTION_MASK;  // Apply direction invert mask
  busy = false;
}

inline void trigger_pulse(uint32_t active){
  // Hand PIT1 the bit mask containing the step pins to toggle and enable it. In the case of stepper drivers
  // that require a delay between setting direction pins and step pins (eg. DRV8825 - 650 ns), the PIT1 interrupt
  // triggers once to set the pins active (per invert mask) and then resets itself and fires again to clear them.
  // Otherwise, we toggle the bits here and pit1 fires once, clearing them.
  pit1_state.active_bits = active;  
#ifdef STEP_PULSE_DELAY
  pit1_state.step_interrupt_status = PULSE_SET;
  PIT_LDVAL1 = STEP_PULSE_DELAY * TICKS_PER_MICROSECOND;
#else
  STEPPER_PORT(TOR) = active;
  PIT_LDVAL1 = pit1_state.pulse_length;
#endif
  PIT_TCTRL1 |= TEN;
}

void pit1_isr(void){
  PIT_TFLG1 = 1;
  PIT_TCTRL1 &= ~TEN;
  STEPPER_PORT(TOR) = pit1_state.active_bits;
#ifdef STEP_PULSE_DELAY
  if(pit1_state.step_interrupt_status == PULSE_SET){
    pit1_state.step_interrupt_status = PULSE_RESET;
    PIT_LDVAL1 = pit1_state.pulse_length;
    PIT_TCTRL1 |= TEN;  
  }
  #endif
}

// Reset and clear stepper subsystem variables
void st_reset()
{
  memset(&st, 0, sizeof(st));
  set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
  current_block = NULL;
  busy = false;
}

// Initialize and start the stepper motor subsystem
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
  SIM_SCGC6 |= SIM_SCGC6_PIT;  //Enable clock input
  PIT_MCR = 0x00;              //Enable PIT, keep running in Debug Mode
  // Configure PIT 0 - main interrupt timer
  PIT_LDVAL0 = 48000000;  //Count Down from this number. 
  // Default to to clock frequency - one full cycle per second.

  PIT_TCTRL0 = TEN; // Keep it running at a reasonable speed, but not interrupting 
  // Configure PIT 1 - reset timer
  PIT_TCTRL1 = TIE;  //enable interrupts

  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);

  // Start in the idle state, but first wake up to check for keep steppers enabled option.
  st_wake_up();
  st_go_idle();
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
// This is rendered trivial by the 32-bit timer!
static uint32_t config_step_timer(uint32_t cycles)
{

  PIT_TCTRL0 &= ~TEN; // Stop the timer 
  PIT_LDVAL0 = cycles; // Load the new value
  PIT_TCTRL0 |= TEN;
  return(cycles);
}

static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE){
    steps_per_minute = MINIMUM_STEPS_PER_MINUTE;
  }
  st.cycles_per_step_event = config_step_timer((F_CPU*((uint32_t)60))/steps_per_minute);
}

// Planner external interface to start stepper interrupt and execute the blocks in queue. Called
// by the main program functions: planner auto-start and run-time command execution.
void st_cycle_start() 
{
  if (sys.state == STATE_QUEUED) {
    sys.state = STATE_CYCLE;
    st_wake_up();
  }
}

// Execute a feed hold with deceleration, only during cycle. Called by main program.
void st_feed_hold() 
{
  if (sys.state == STATE_CYCLE) {
    sys.state = STATE_HOLD;
    sys.auto_start = false; // Disable planner auto start upon feed hold.
  }
}

// Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
// runtime command execution in the main program, ensuring that the planner re-plans safely.
// NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
// cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
// Only the planner de/ac-celerations profiles and stepper rates have been updated.
void st_cycle_reinitialize()
{
  if (current_block != NULL) {
    // Replan buffer from the feed hold stop location.
    plan_cycle_reinitialize(current_block->step_event_count - st.step_events_completed);
    // Update initial rate and timers after feed hold.
    st.trapezoid_adjusted_rate = 0; // Resumes from rest
    set_step_events_per_minute(st.trapezoid_adjusted_rate);
    st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
    st.step_events_completed = 0;
    sys.state = STATE_QUEUED;
  } else {
    sys.state = STATE_IDLE;
  }
}
