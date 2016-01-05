#include <avr/io.h>
#include "host.h"
#include "host_driver.h"
#include "serial.h"
#include "fb155bc.h"
#include "print.h"
#include "timer.h"
#include "wait.h"


/* Host driver */
static uint8_t keyboard_leds(void);
static void send_keyboard(report_keyboard_t *report);
static void send_mouse(report_mouse_t *report);
static void send_system(uint16_t data);
static void send_consumer(uint16_t data);

host_driver_t fb155bc_driver = {
    keyboard_leds,
    send_keyboard,
    send_mouse,
    send_system,
    send_consumer
};


void fb155bc_init(void)
{
    serial_init();
    print("\nInitialising...\n");
    // wait for keyboard to boot up and receive command
    DDRD   = 0b01001000;//_BV(PD6);
    PORTD  = 0b00001000;//_BV(PD6);

    PORTD |= _BV(PD6);
    _delay_ms(1000);

    PORTD &= ~(_BV(PD6));
    _delay_ms(500);

    PORTD |= _BV(PD6);
    _delay_ms(500);

    char reset[] = "ATZ\r";
    for (int i = 0; i < 5; i++) {
        serial_send(reset[i]);
    }
    _delay_ms(500);

    char ble_enable[] = "AT+BTSCAN\r";
    for (int i = 0; i < 11; i++) {
        serial_send(ble_enable[i]);
    }
    //_delay_ms(1000);


    /*// JTAG disable for PORT F. write JTD bit twice within four cycles.
    MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD);

    // PF7: BT connection control(high: connect, low: disconnect)
    fb155bc_autoconnect();

    // PF6: linked(input without pull-up)
    DDRF  &= ~(1<<6);
    PORTF |=  (1<<6);

    // PF1: RTS(low: allowed to send, high: not allowed)
    DDRF &= ~(1<<1);
    PORTF &= ~(1<<1);

    // PD5: CTS(low: allow to send, high:not allow)
    DDRD |= (1<<5);
    PORTD &= ~(1<<5);

    serial_init(); */

}

int16_t fb155bc_getc(void)
{
    return serial_recv2();
}

const char *fb155bc_gets(uint16_t timeout)
{
    static char s[24];
    uint16_t t = timer_read();
    uint8_t i = 0;
    int16_t c;
    while (i < 23 && timer_elapsed(t) < timeout) {
        if ((c = fb155bc_getc()) != -1) {
            if ((char)c == '\r') continue;
            if ((char)c == '\n') break;
            s[i++] = c;
        }
    }
    s[i] = '\0';
    return s;
}

void fb155bc_putc(uint8_t c)
{
    //printf(c);
    serial_send(c);
}

void fb155bc_puts(char *s)
{
    while (*s){
        //printf(s);
	serial_send(*s++);
    }
}

bool fb155bc_autoconnecting(void)
{
    // GPIO6 for control connection(high: auto connect, low: disconnect)
    // Note that this needs config: SM,4(Auto-Connect DTR Mode)
    print("\nautoconnecting...\n");
    return (PORTF & (1<<7) ? true : false);
}

void fb155bc_autoconnect(void)
{
    print("\nautoconnect\n");
    char connect[] = "ATD\r";
    for (int i = 0; i < 5; i++) {
        serial_send(connect[i]);
    }
}

void fb155bc_disconnect(void)
{
    print("\nDisconnecting...\n");
    DDRD   = 0b01001000;//_BV(PD6);
    PORTD  = 0b00001000;//_BV(PD6);

    PORTD |= _BV(PD6);

    _delay_ms(100);

    PORTD &= ~(_BV(PD6));

    _delay_ms(50);

    PORTD |= _BV(PD6);

    _delay_ms(50);

    char reset[] = "ATZ\r";
    for (int i = 0; i < 5; i++) {
        serial_send(reset[i]);
    }
    //host_set_driver(&lufa_driver);
}

bool fb155bc_rts(void)
{
    // low when RN-42 is powered and ready to receive
    if(PIND&(1<<2))
    {
        print("\nRTS HI\n");
    } else
    {
        print("\nRTS LO\n");
    }
    return !PIND&(1<<2);
}

void fb155bc_cts_hi(void)
{
    // not allow to send
    //PORTD |= (1<<5);
}

void fb155bc_cts_lo(void)
{
    // allow to send
    //PORTD &= ~(1<<5);
}

bool fb155bc_linked(void)
{
    // fb155bc GPIO2
    //   Hi-Z:  Not powered
    //   High:  Linked
    //   Low:   Connecting
    if(PIND&(1<<2))
    {
        print("\nLINKED\n");
    } else
    {
        print("\nUNLINKED\n");
    }
    return !PIND&(1<<2);
}


static uint8_t leds = 0;
static uint8_t keyboard_leds(void) { return leds; }
void fb155bc_set_leds(uint8_t l) { leds = l; }

static void send_keyboard(report_keyboard_t *report)
{
    // wake from deep sleep
/*
    PORTD |= (1<<5);    // high
    wait_ms(5);
    PORTD &= ~(1<<5);   // low
*/
    print("\nSend keyboard\n");
    serial_send(0x0c);  // Length of packet in bytes
    serial_send(0x00);     // Type of packet: Forward HID Report
    serial_send(0xa1);     // HID input Report Header
    serial_send(0x01);     // Keyboard Report ID 1
    serial_send(report->mods); // Modifier Keys
    serial_send(0x00); // Reserved
    serial_send(report->keys[0]); //Keycode 1
    serial_send(report->keys[1]); //Keycode 2
    serial_send(report->keys[2]); //Keycode 3
    serial_send(report->keys[3]); //Keycode 4
    serial_send(report->keys[4]); //Keycode 5
    serial_send(report->keys[5]); //Keycode 6
}

static void send_mouse(report_mouse_t *report)
{
    // wake from deep sleep
/*
    PORTD |= (1<<5);    // high
    wait_ms(5);
    PORTD &= ~(1<<5);   // low
*/

    serial_send(0x08); // Length of packet in bytes
    serial_send(0x00); // Type of packet: Forward HID Report
    serial_send(0xa1); //HID input Report Header
    serial_send(0x02);     // Keyboard Report ID 2
    serial_send(report->buttons); //Up to 5 buttons, one bit each
    serial_send(report->x); //Delta X
    serial_send(report->y); //Delta Y
    serial_send(report->v); //Wheel
}

static void send_system(uint16_t data)
{
    // Table 5-6 of RN-BT-DATA-UB
    // 81,82,83 scan codes can be used?
}


static uint16_t usage2bits(uint16_t usage)
{
    switch (usage) {
        case AC_HOME:                 return 0x01;
        case AL_EMAIL:                return 0x02;
        case AC_SEARCH:               return 0x04;
        //case AL_KBD_LAYOUT:         return 0x08;  // Apple virtual keybaord toggle
        case AUDIO_VOL_UP:            return 0x10;
        case AUDIO_VOL_DOWN:          return 0x20;
        case AUDIO_MUTE:              return 0x40;
        case TRANSPORT_PLAY_PAUSE:    return 0x80;
        case TRANSPORT_NEXT_TRACK:    return 0x100;
        case TRANSPORT_PREV_TRACK:    return 0x200;
        case TRANSPORT_STOP:          return 0x400;
        case TRANSPORT_STOP_EJECT:    return 0x800;
        case TRANSPORT_FAST_FORWARD:  return 0x1000;
        case TRANSPORT_REWIND:        return 0x2000;
        //case return 0x4000;   // Stop/eject
        //case return 0x8000;   // Internet browser
    };
    return 0;
}

static void send_consumer(uint16_t data)
{
    uint16_t bits = usage2bits(data);
    serial_send(0xFD);  // Raw report mode
    serial_send(3);     // length
    serial_send(3);     // descriptor type
    serial_send(bits&0xFF);
    serial_send((bits>>8)&0xFF);
}


/* Null driver for config_mode */
static uint8_t config_keyboard_leds(void);
static void config_send_keyboard(report_keyboard_t *report);
static void config_send_mouse(report_mouse_t *report);
static void config_send_system(uint16_t data);
static void config_send_consumer(uint16_t data);

host_driver_t fb155bc_config_driver = {
    config_keyboard_leds,
    config_send_keyboard,
    config_send_mouse,
    config_send_system,
    config_send_consumer
};

static uint8_t config_keyboard_leds(void) { return leds; }
static void config_send_keyboard(report_keyboard_t *report) {}
static void config_send_mouse(report_mouse_t *report) {}
static void config_send_system(uint16_t data) {}
static void config_send_consumer(uint16_t data) {}
