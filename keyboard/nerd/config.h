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
#ifndef CONFIG_H
#define CONFIG_H

#ifdef BLUETOOTH_ENABLE
	#ifdef __AVR_ATmega32U4__
	    #define SERIAL_UART_BAUD       38400 //The default of 115200 won't work with an 8 MHz FCPU
	    #define SERIAL_UART_DATA       UDR1
	    #define SERIAL_UART_UBRR       ((F_CPU/(16UL*SERIAL_UART_BAUD))-1)
	    #define SERIAL_UART_RXD_VECT   USART1_RX_vect
	    #define SERIAL_UART_TXD_READY  (UCSR1A&(1<<UDRE1))
	    #define SERIAL_UART_INIT()     do { \
	        UBRR1L = (uint8_t) SERIAL_UART_UBRR;       /* baud rate */ \
	        UBRR1H = (uint8_t) (SERIAL_UART_UBRR>>8);  /* baud rate */ \
	        UCSR1B = (1<<TXEN1);                /* TX: enable */ \
	        UCSR1C = (0<<UPM11) | (0<<UPM10) | /* parity: none(00), even(01), odd(11) */ \
	                 (0<<UCSZ12) | (1<<UCSZ11) | (1<<UCSZ10); /* data-8bit(011) */ \
			sei(); \
	    } while(0)
	#else
	#   error "USART configuration is needed."
	#endif
#endif

/* USB Device descriptor parameter */
#define VENDOR_ID       0xFEED
#define PRODUCT_ID      0x6060
#define DEVICE_VER      0x0001
#define MANUFACTURER    GON
#define PRODUCT         NerD
#define DESCRIPTION     t.m.k. keyboard firmware for NerD

/* key matrix size */
#define MATRIX_ROWS 9
#define MATRIX_COLS 10

/* Set 0 if debouncing isn't needed */
#define DEBOUNCE 5

/* number of backlight levels */
#define BACKLIGHT_LEVELS 4

/* Mechanical locking support. Use KC_LCAP, KC_LNUM or KC_LSCR instead in keymap */
#define LOCKING_SUPPORT_ENABLE

/* Locking resynchronize hack */
#define LOCKING_RESYNC_ENABLE

/* key combination for command */
#define IS_COMMAND() ( \
    keyboard_report->mods == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)) \
)

#endif
