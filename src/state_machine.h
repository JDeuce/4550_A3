#ifndef _STATE_MACHINE_H
#define _STATE_MACHINE_H
#include <avr/io.h>

#ifndef NULL
#define NULL 0
#endif
// ==================
// Structures for state machine
// ==================
struct State;
struct Transition {
    unsigned char mask;
    // masks is a bit flag with 4 bits in use
    // 0x01 = SW0 is pressed
    // 0x02 = SW1 is pressed
    // 0x10 = SW0 is up
    // 0x20 = SW1 is up

    unsigned int min_time; // minimum time to transition
    unsigned int max_time; // maximum time to transition
    unsigned int next_state_id;
    void (*output)(void);

};

struct State {
    unsigned int state_id;
    unsigned char num_transitions;
    struct Transition transitions[5];
};

struct StateMachine {
    struct State *states;
    uint8_t state;
    uint32_t last_transition;
};

struct Transition *get_pending_transition(struct State *state, unsigned char input, uint32_t current_time);
void update_state_machine(struct StateMachine *machine, uint32_t clock, unsigned char input);
#endif
