#ifndef FB155BC_H
#define FB155BC_H

#include <stdbool.h>

host_driver_t fb155bc_driver;
host_driver_t fb155bc_config_driver;

void fb155bc_init(void);
int16_t fb155bc_getc(void);
const char *fb155bc_gets(uint16_t timeout);
void fb155bc_putc(uint8_t c);
void fb155bc_puts(char *s);
bool fb155bc_autoconnecting(void);
void fb155bc_autoconnect(void);
void fb155bc_disconnect(void);
bool fb155bc_rts(void);
void fb155bc_cts_hi(void);
void fb155bc_cts_lo(void);
bool fb155bc_linked(void);
void fb155bc_set_leds(uint8_t l);

#endif
