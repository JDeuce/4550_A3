#ifndef _HYSTERISIS_H
#define _HYSTERISIS_H
#include <avr/io.h>
// ================
// Structures for hysterisis
// ================
struct HysterisisConfig {
    unsigned char min;
    unsigned char max;
    unsigned char on;
    unsigned char off;
};

enum ButtonState {
    BUTTON_UP,
    BUTTON_DOWN
};

struct HysterisisState {
    unsigned char count;
    unsigned char mask;
    enum ButtonState state;
    struct HysterisisConfig settings;
};

void hysterisis_poll(uint8_t input, struct HysterisisState *button);
#endif
