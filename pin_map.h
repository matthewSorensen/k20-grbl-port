/*
  pin_map.h - Pin mapping configuration file
  Part of Grbl

  Copyright (c) 2014 Matthew D. Sorensen


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

#ifndef pin_map_h
#include <mk20dx128.h>

#define pin_map_h

#define SPINDLE_PORT(reg) GPIOA_P##reg
#define SPINDLE_DDR GPIOA_PDDR

#define SPINDLE_ENABLE_CTRL PORTA_PCR12
#define SPINDLE_ENABLE_BIT  (1<<12)
#define SPINDLE_DIRECTION_CRL PORTA_PCR13
#define SPINDLE_DIRECTION_BIT (1<<13)

#define COOLANT_PORT(reg) GPIOD_P##reg
#define COOLANT_DDR GPIOD_PDDR

#define COOLANT_FLOOD_CTRL PORTD_PCR7
#define COOLANT_FLOOD_BIT  (1<<7)
#define COOLANT_MIST_CTRL  PORTD_PCR4
#define COOLANT_MIST_BIT   (1<<4)


#define STEPPER_DISABLE_PORT(reg) GPIOC_P##reg
#define STEPPER_DISABLE_DDR       GPIOC_PDDR
#define STEPPER_DISABLE_CTRL      PORTC_PCR1
#define STEPPER_DISABLE_BIT       2  //Teensy3.x pin 22


#define STEPPER_PORT(reg) GPIOC_P##reg
#define STEPPER_DDR GPIOC_PDDR   

#define STEP_X_CTRL PORTC_PCR3   
#define STEP_X_BIT  (1<<3)        //Teensy3.x pin 9
#define STEP_Y_CTRL PORTC_PCR4
#define STEP_Y_BIT  (1<<4)        //Teensy3.x pin 10
#define STEP_Z_CTRL PORTC_PCR6
#define STEP_Z_BIT  (1<<6)        //Teensy3.x pin 11   

#define DIR_X_CTRL PORTC_PCR7
#define DIR_X_BIT  (1<<7)        //Teensy3.x pin 12   
#define DIR_Y_CTRL PORTC_PCR5
#define DIR_Y_BIT  (1<<5)        //Teensy3.x pin 13   
#define DIR_Z_CTRL PORTC_PCR0
#define DIR_Z_BIT  1             //Teensy3.x pin 15   

#define DIRECTION_MASK (DIR_X_BIT | DIR_Y_BIT | DIR_Z_BIT)
#define STEP_MASK (STEP_X_BIT | STEP_Y_BIT | STEP_Z_BIT)


#define LIMIT_PORT(reg) GPIOB_P##reg
#define LIMIT_DDR       GPIOB_PDDR

#define LIMIT_X_CTRL PORTB_PCR0
#define LIMIT_X_BIT  1               //Teensy3.x pin 16   
#define LIMIT_Y_CTRL PORTB_PCR1
#define LIMIT_Y_BIT  2               //Teensy3.x pin 17   
#define LIMIT_Z_CTRL PORTB_PCR3
#define LIMIT_Z_BIT  (1<<3)         //Teensy3.x pin 18   
#define LIMITS_MASK (3 | (1<<3))

#define RESET_CTRL PORTB_PCR16
#define RESET_BIT  (1<<16)
#define FEED_HOLD_CTRL PORTB_PCR17
#define FEED_HOLD_BIT (1<<17)
#define CYCLE_CTRL PORTB_PCR19
#define CYCLE_BIT (1<<19)

#define INPUT_MASK (RESET_BIT | FEED_HOLD_BIT | CYCLE_BIT)


#endif
