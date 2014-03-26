#include "state_machine.h"
// ===============
// State machine functions
// ===============

// given current state, input value, and the time since transitioning, this
// function returns NULL if no state transition should be made,
//                                  or a pointer to a transition if one should be made
struct Transition *get_pending_transition(struct State *state, unsigned char input, uint32_t current_time) {
    int i;
    struct Transition *t;
    for (i = 0; i < state->num_transitions; i++) {
        t = &state->transitions[i];

        if (current_time >= t->min_time) {
            if (t->max_time == 0 ||
                    current_time < t->max_time) {
                // time constraints met, does input match
                // transition mask
                if ((t->mask & input) == t->mask) {
                    return t;
                }
            }
        }
    }

    return NULL;
}

// updates the state machine based on new input
void update_state_machine(struct StateMachine *machine, uint32_t clock, unsigned char input)
{
    struct Transition *next;
    uint32_t time_since_trans;
    struct State *state = &machine->states[machine->state];
    time_since_trans = clock - machine->last_transition;
    next = get_pending_transition(state, input, time_since_trans);

    if (next != NULL) {
        if (next->output != NULL)
            next->output();

        machine->state =next->next_state_id;
        machine->last_transition = clock;
    }
}
