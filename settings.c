/*
  settings.c - eeprom configuration handling 
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

#include "protocol.h"
#include "report.h"
#include "stepper.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "limits.h"

settings_t settings;

#include <flexram.h>

// Method to store startup lines into EEPROM
void settings_store_startup_line(uint8_t n, char *line)
{
  uint32_t addr = n*(LINE_BUFFER_SIZE/4+1)+EEPROM_ADDR_STARTUP_BLOCK;
  write_with_checksum((uint32_t*) line, addr, LINE_BUFFER_SIZE/4);
}

// Method to store coord data parameters into EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{  
  uint32_t addr = coord_select*(N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  write_with_checksum((uint32_t*) coord_data, addr, N_AXIS);
}  

// Method to store Grbl global settings struct and version number into EEPROM
void write_global_settings() 
{
  write_with_checksum((uint32_t*) &settings, EEPROM_ADDR_GLOBAL, (sizeof(settings_t)+3) >> 2);
}


// Method to reset Grbl global settings back to defaults. 
void settings_reset(void){
  // Reset all settings or only the migration settings to the new version.
  settings.version = SETTINGS_VERSION;
  settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
  settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
  settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
  settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
  settings.default_feed_rate = DEFAULT_FEEDRATE;
  settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
  settings.acceleration = DEFAULT_ACCELERATION;
  settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
  settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
  
  settings.flags = 0;
  if (DEFAULT_REPORT_INCHES) { settings.flags |= BITFLAG_REPORT_INCHES; }
  if (DEFAULT_AUTO_START) { settings.flags |= BITFLAG_AUTO_START; }
  if (DEFAULT_INVERT_ST_ENABLE) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
  if (DEFAULT_HARD_LIMIT_ENABLE) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
  if (DEFAULT_HOMING_ENABLE) { settings.flags |= BITFLAG_HOMING_ENABLE; }
  settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
  settings.homing_feed_rate = DEFAULT_HOMING_FEEDRATE;
  settings.homing_seek_rate = DEFAULT_HOMING_RAPID_FEEDRATE;
  settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
  settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;
  settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
  settings.decimal_places = DEFAULT_DECIMAL_PLACES;
  settings.n_arc_correction = DEFAULT_N_ARC_CORRECTION;
  write_global_settings();
}

// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
  uint32_t addr = n*(LINE_BUFFER_SIZE/4+1)+EEPROM_ADDR_STARTUP_BLOCK; 
  if(read_with_checksum((uint32_t*) line, addr, LINE_BUFFER_SIZE/4)){
    // Reset line with default value
    line[0] = 0;
    settings_store_startup_line(n, line);
    return(false);
  } else {
    return(true);
  }
}

// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  uint32_t addr = coord_select*(N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  if(read_with_checksum((uint32_t*) coord_data, addr, N_AXIS)){
    clear_vector_float(coord_data);
    settings_write_coord_data(coord_select,coord_data);
    return (false);
  }else{
    return (true);
  }
}  

// Reads Grbl global settings struct from EEPROM.
uint8_t read_global_settings(void){
  if(read_with_checksum((uint32_t*) &settings, EEPROM_ADDR_GLOBAL, (sizeof(settings_t)+3) >> 2)){
    return false;
  }else{
    return true;
  }
}


// A helper method to set settings from command line
uint8_t settings_store_global_setting(int parameter, float value) {
  switch(parameter) {
    case 0: case 1: case 2:
      if (value <= 0.0) { return(STATUS_SETTING_VALUE_NEG); } 
      settings.steps_per_mm[parameter] = value; break;
    case 3: 
      if (value < 3) { return(STATUS_SETTING_STEP_PULSE_MIN); }
      settings.pulse_microseconds = round(value); break;
    case 4: settings.default_feed_rate = value; break;
    case 5: settings.default_seek_rate = value; break;
    case 6: settings.invert_mask = trunc(value); break;
    case 7: settings.stepper_idle_lock_time = round(value); break;
    case 8: settings.acceleration = value*60*60; break; // Convert to mm/min^2 for grbl internal use.
    case 9: settings.junction_deviation = fabs(value); break;
    case 10: settings.mm_per_arc_segment = value; break;
    case 11: settings.n_arc_correction = round(value); break;
    case 12: settings.decimal_places = round(value); break;
    case 13:
      if (value) { settings.flags |= BITFLAG_REPORT_INCHES; }
      else { settings.flags &= ~BITFLAG_REPORT_INCHES; }
      break;
    case 14: // Reset to ensure change. Immediate re-init may cause problems.
      if (value) { settings.flags |= BITFLAG_AUTO_START; }
      else { settings.flags &= ~BITFLAG_AUTO_START; }
      break;
    case 15: // Reset to ensure change. Immediate re-init may cause problems.
      if (value) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
      else { settings.flags &= ~BITFLAG_INVERT_ST_ENABLE; }
      break;
    case 16:
      if (value) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
      else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
      limits_init(); // Re-init to immediately change. NOTE: Nice to have but could be problematic later.
      break;
    case 17:
      if (value) { settings.flags |= BITFLAG_HOMING_ENABLE; }
      else { settings.flags &= ~BITFLAG_HOMING_ENABLE; }
      break;
    case 18: settings.homing_dir_mask = trunc(value); break;
    case 19: settings.homing_feed_rate = value; break;
    case 20: settings.homing_seek_rate = value; break;
    case 21: settings.homing_debounce_delay = round(value); break;
    case 22: settings.homing_pulloff = value; break;
    default: 
      return(STATUS_INVALID_STATEMENT);
  }
  write_global_settings();
  return(STATUS_OK);
}

// Initialize the config subsystem
void settings_init() {
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_reset();
    report_grbl_settings();
  }
  // Read all parameter data into a dummy variable. If error, reset to zero, otherwise do nothing.
  float coord_data[N_AXIS];
  uint8_t i;
  for (i=0; i<=SETTING_INDEX_NCOORD; i++) {
    if (!settings_read_coord_data(i, coord_data)) {
      report_status_message(STATUS_SETTING_READ_FAIL);
    }
  }
  // NOTE: Startup lines are handled and called by main.c at the end of initialization.
}
