/*
  print.c - Functions for formatting output strings
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

#include "config.h"
#include "settings.h"

int usb_serial_putchar(uint8_t c); 
int usb_serial_write(const void *buffer, uint32_t size);

void printString(const char *s)
{
  uint32_t len = 0;
  const char* p = s;
  while(*(p++)) len++;
  
  usb_serial_write(s, len);
}

void print_uint8_base2(uint8_t n)
{ 
	unsigned char buf[8];
	uint8_t i = 0;
	
	for (; i < 8; i++) {
	  buf[i] = (n & 1) + '0';
	  n >>= 1;
	}
	usb_serial_write(buf, 8);
}

void print_uint32_base10(uint32_t n)
{ 
  unsigned char buf[10]; 
  uint8_t i = 0;
  
  if (n == 0) {
    usb_serial_putchar('0');
    return;
  } 
  
  while (n > 0) {
    buf[i++] = n % 10 + '0';
    n /= 10;
  }
    
  for (; i > 0; i--)
    usb_serial_putchar(buf[i-1]);
}

void printInteger(long n)
{
  if (n < 0) {
    usb_serial_putchar('-');
    n = -n;
  }
  print_uint32_base10(n);
}

// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
// NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up 
// techniques are actually just slightly slower. Found this out the hard way.
void printFloat(float n)
{
  if (n < 0) {
    usb_serial_putchar('-');
    n = -n;
  }

  uint8_t decimals = settings.decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value.
    
  // Generate digits backwards and store in string.
  unsigned char buf[10]; 
  uint8_t i = 0;
  uint32_t a = (long)n;  
  buf[settings.decimal_places] = '.'; // Place decimal point, even if decimal places are zero.
  while(a > 0) {
    if (i == settings.decimal_places) { i++; } // Skip decimal point location
    buf[i++] = (a % 10) + '0'; // Get digit
    a /= 10;
  }
  while (i < settings.decimal_places) { 
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
  }
  if (i == settings.decimal_places) { // Fill in leading zero, if needed.
    i++;
    buf[i++] = '0'; 
  }   
  
  // Print the generated string.
  for (; i > 0; i--)
    usb_serial_putchar(buf[i-1]);
}
