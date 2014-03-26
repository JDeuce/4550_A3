#include "hysterisis.h"

// ==================
// Hysterisis functions
// ==================
// updates the hysterisis state based on the current input on PIND
void hysterisis_poll(uint8_t input, struct HysterisisState *button) {
    // change the count
    if (input & button->mask) {

        if (button->count > button->settings.min)
            button->count--;
    } else {
        if (button->count < button->settings.max)
            button->count++;
    }

    // check for a state change
    if (button->count >= button->settings.on)
        button->state = BUTTON_DOWN;
    else if (button->count <= button->settings.off)
        button->state = BUTTON_UP;
}

