#include "power_test.h"
#include <stdio.h>
void pt_port_can_send(uint32_t id, const uint8_t d[8]) { (void)id; (void)d; }
uint32_t pt_port_now_ms(void) { return 0; }
void pt_port_log(const char *s) { fputs(s, stdout); }
void pt_port_write(const uint8_t *b, uint16_t n) { (void)b; (void)n; }
void pt_port_read_bus(uint16_t *mv, int32_t *ma) { *mv = 24000; *ma = 120; }
int main(void) {
    pt_config_t c = {1, 2000, 0, 1000, 100, 5, 9000, 80};
    uint8_t fb[8] = {0x10,0x20,0x0F,0xA0,0x00,0x64,0x1E,0};
    pt_init(&c); pt_start();
    pt_on_can_rx(0x201, fb, 8);
    for (int i = 0; i < 20; i++) pt_tick_1ms();
    return 0;
}
