/*
Copyright 2014 Ralf Schmitt <ralf@bunkertor.net>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include "backlight.h"
#include "led.h"

#define CHANNEL_SWITCH  OCR1C
#define CHANNEL_PCB     OCR1B

void backlight_init_ports()
{
    DDRB |= 0b11100000; // PB7 (switch), PB6 (pcb), PB5 (caps)
    PORTB &= ~(1<<7);
    
    // Use full 16-bit resolution. 
    ICR1 = 0xFFFF;

    // I could write a wall of text here to explain... but TL;DW
    // Go read the ATmega32u4 datasheet.
    // And this: http://blog.saikoled.com/post/43165849837/secret-konami-cheat-code-to-high-resolution-pwm-on
    
    // Pin PB7 = OCR1C (Timer 1, Channel C)
    // Compare Output Mode = Clear on compare match, Channel C = COM1C1=1 COM1C0=0
    // (i.e. start high, go low when counter matches.)
    // WGM Mode 14 (Fast PWM) = WGM13=1 WGM12=1 WGM11=1 WGM10=0
    // Clock Select = clk/1 (no prescaling) = CS12=0 CS11=0 CS10=1
    
    TCCR1A = _BV(COM1C1) | _BV(WGM11); // = 0b00001010;
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // = 0b00011001;

    backlight_init();
}

void led_set(uint8_t usb_led)
{
    (usb_led & (1<<USB_LED_CAPS_LOCK)) ? backlight_caps_enable() : backlight_caps_disable();
}

/* 
//void backlight_switch_set(uint8_t level)
void backlight_set(uint8_t level)
{
    //(level & BACKLIGHT_SWITCH) ? backlight_switch_enable() : backlight_switch_disable();
    //(level & BACKLIGHT_PCB) ? backlight_pcb_enable() : backlight_pcb_disable();
    if ( level == 0 )
    {
        // Turn off PWM control on PB7, revert to output low.
        TCCR1A &= ~(_BV(COM1C1));
        CHANNEL_SWITCH = 0x0;
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB7));
    }
    else if ( level == BACKLIGHT_LEVELS )
    {
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB7));
        // Turn on PWM control of PB7
        TCCR1A |= _BV(COM1C1);
        // Set the brightness
        CHANNEL_SWITCH = 0xFFFF;
    }
    else        
    {
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB7));
        // Turn on PWM control of PB7
        TCCR1A |= _BV(COM1C1);
        // Set the brightness
        CHANNEL_SWITCH = 0xFFFF >> ((BACKLIGHT_LEVELS - level) * ((BACKLIGHT_LEVELS + 1) / 2));
    }
}
*/
void backlight_set(uint8_t level)
//void backlight_pcb_set(uint8_t level)
{
    //(level & BACKLIGHT_SWITCH) ? backlight_switch_enable() : backlight_switch_disable();
    //(level & BACKLIGHT_PCB) ? backlight_pcb_enable() : backlight_pcb_disable();
    if ( level == 0 )
    {
        // Turn off PWM control on PB6, revert to output low.
        TCCR1A &= ~(_BV(COM1B1));
        CHANNEL_PCB = 0x0;
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB6));
    }
    else if ( level == BACKLIGHT_LEVELS )
    {
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB6));
        // Turn on PWM control of PB6
        TCCR1A |= _BV(COM1B1);
        // Set the brightness
        CHANNEL_PCB = 0xFFFF;
    }
    else        
    {
        // Prevent backlight blink on lowest level
        PORTB &= ~(_BV(PORTB6));
        // Turn on PWM control of PB6
        TCCR1A |= _BV(COM1B1);
        // Set the brightness
        CHANNEL_PCB = 0xFFFF >> ((BACKLIGHT_LEVELS - level) * ((BACKLIGHT_LEVELS + 1) / 2));
    }
} 

void backlight_switch_enable()
{
    PORTB |= 0b10000000;
}

void backlight_switch_disable()
{
    PORTB &= ~0b10000000;
}

void backlight_switch_invert()
{
    PORTB ^= 0b10000000;
}

void backlight_pcb_enable()
{
    PORTB |= 0b01000000;
}

void backlight_pcb_disable()
{
    PORTB &= ~0b01000000;
}

void backlight_pcb_invert()
{
    PORTB ^= 0b01000000;
}

void backlight_caps_enable()
{
    PORTB |= 0b00100000;
}

void backlight_caps_disable()
{
    PORTB &= ~0b00100000;
}

void backlight_caps_invert()
{
    PORTB ^= 0b00100000;
}
