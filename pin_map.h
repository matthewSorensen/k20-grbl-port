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

#endif
