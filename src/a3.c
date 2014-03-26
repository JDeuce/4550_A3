/* Name: Josh Jaques
 * Studno: 6822808
 * Course: COMP 4550
 * Instructor: Zapp
 * Assignment: 3 Question: 1
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#include "state_machine.h"
#include "hysterisis.h"

#define SWITCH_PIN      PINC
#define SWITCH_DIR      DDRC
#define LED_PORT	PORTB
#define LED_DIR		DDRB

#define DASH_TIME 250
#define WAIT_TIME 1000

#define LED_DASH 1000
#define LED_DOT  250
#define LED_WAIT 250

#define DOT 0
#define DASH 1

#define DUTY_IDLE	0
#define DUTY_START	50
#define DUTY_DOT 	100
#define DUTY_DASH	150
#define DUTY_STOP	200
#define DUTY_TOLERANCE 20

// masks for input state machine
#define SW1_DOWN 0x01
#define SW1_UP   0x10

// masks for displaystate machine
#define DISP_RDY 0x10
#define DISP_COMPLETE 0x01
#define DISP_DOT 0x02
#define DISP_DASH 0x20
#define EPSILON  0

#define TIMER0_SELECT	6
#define TX_DIR  DDRD
#define TX_PORT	PORTD
#define TX_PIN 	_BV(PD7)
#define RX_PORT PIND
#define RX_DIR  DDRD
#define RX_PIN  _BV(PD2)

#define TURN_INPUT_TX 0
#define TURN_DISPLAY_RX 1

#define BUFF_SIZE 16

struct Buffer {
    uint16_t value;
    uint8_t bit;
    uint8_t n_bits;
};
// ================
// buffers for xmit/recv
// ================
static struct Buffer tx_buffer = { 0, 0, 0 };
static struct Buffer rx_buffer = { 0, 0, 0 };
static struct Buffer disp_buffer = {0, 0, 0};
static struct Buffer input_buffer = { 0, 0, 0 };

// push a bit onto a buffer
int buffer_push_bit(struct Buffer *buff, uint8_t bit)
{
    bit &= 0x01;
    if (buff->n_bits < BUFF_SIZE) {
        buff->value |= (bit << buff->bit);
        buff->bit++;
        buff->n_bits++;
        return 1;
    } else {
        return 0;
    }
}

// pop a bit from a buffer
char buffer_pop_bit(struct Buffer *buff)
{
    if (buff->bit < buff->n_bits) {
        uint16_t bit = buff->value & (1 << buff->bit);
        buff->bit++;
        if (bit)
            return 1;
        else
            return 0;
    } else {
        return -1;
    }
}

// reset a buffer
void buff_reset(struct Buffer *dest)
{
    dest->bit = 0;
    dest->n_bits = 0;
    dest->value = 0;
}

// copy a buffer
void buff_set(struct Buffer *dest, struct Buffer *src)
{
    dest->bit = 0;
    dest->n_bits = src->n_bits;
    dest->value = src->value;
}

// ==============
// input State machine outputs
// ==============
void input_dot()
{
    buffer_push_bit(&input_buffer, DOT);
}

void input_dash()
{
    buffer_push_bit(&input_buffer, DASH);
}

// ==============
// input State machine transitions
// ==============
enum INPUT_STATES {
    INPUT_IDLE,
    WAIT_NEXT,
    DOT_OR_DASH,
    TRANSMIT
};

static struct State input_states[] = {
    {
        // STATE,       # TRANS
        INPUT_IDLE,		1,
        {
            // MASK   	MIN     MAX     NEXT            OUTPUT
            { SW1_DOWN,	0,		0,		DOT_OR_DASH,	NULL }
        }
    },
    {
        WAIT_NEXT,          2,
        {
            { SW1_DOWN,			0,      0,	DOT_OR_DASH,    NULL },
            { SW1_UP,	WAIT_TIME,      0,	TRANSMIT, 	NULL },
        }
    },
    {
        DOT_OR_DASH,	2,
        {
            { SW1_UP,	0,		DASH_TIME,	WAIT_NEXT,	input_dot },
            { SW1_UP,	DASH_TIME, 	0,		WAIT_NEXT,	input_dash },
        }
    },
    {
        TRANSMIT, 0, { }
    }


};

static struct StateMachine input_state_machine = {
    .states = input_states,
    .state = INPUT_IDLE,
    .last_transition = 0
};

// ==============
// display state machine outputs
// ==============
static uint8_t last_displayed;
void display_bit_on_leds()
{
    char bit = buffer_pop_bit(&disp_buffer);
    if (bit >= 0) {
        last_displayed = bit;
        LED_PORT = 0xFD;
    }
}

void clear_leds()
{
    LED_PORT = 0xFF;
}

// ==============
// display state machine transitions
// ==============
enum DISPLAY_STATES {
    DISP_IDLE,
    DISPLAY_START,
    SHOW_BIT,
    CLEAR_BIT
};

static struct State display_states[] = {
    {
        DISP_IDLE,		0,
        { }
    },
    {
        DISPLAY_START,	2,
        {
            { DISP_RDY,		0,	0,	SHOW_BIT,	display_bit_on_leds },
            { DISP_COMPLETE,	0,	0,	DISP_IDLE,	clear_leds }
        }
    },
    {
        SHOW_BIT,	2,
        {
            { DISP_DOT,	LED_DOT,	0,	CLEAR_BIT,	clear_leds },
            { DISP_DASH,	LED_DASH,	0,	CLEAR_BIT,	clear_leds },
        }
    },
    {
        CLEAR_BIT,	1,
        {
            { EPSILON,	LED_WAIT,	0,	DISPLAY_START,	NULL }
        }
    }
};

static struct StateMachine display_state_machine = {
    .states = display_states,
    .state = DISP_IDLE,
    .last_transition = 0
};

// ==================
// button states
// ==================
static struct HysterisisState SW1_STATE = {
    .count = 0,
    .state = BUTTON_UP,
    .mask = _BV(PD1),
    .settings = { .min = 0, .max = 3, .on = 2, .off = 1 }
};


// ===============
// Timer code
// ===============
static volatile uint32_t elapsed_ms = 0; // used to track time

// clock overflow
ISR (TIMER1_OVF_vect) {
    TCNT1 = 64536;  // normalize with 1mhz clock rate so that 16bit timer overflows again in 1ms
    elapsed_ms++;
}

void setup_timer1() {
    TCNT1 = 64536;       // set initial timer count
    TCCR1B |= _BV(CS00); // use internal clock for timer0
    TIMSK |= _BV(TOIE1); // enable timer0 overflow interrupt
}

// disable the jtag function of PORTC
void disable_jtag()
{
    unsigned char sreg = SREG;
    cli();
    MCUCSR |= _BV(JTD);
    MCUCSR |= _BV(JTD);
    SREG = sreg;
}


// turns the hysterisis states of the buttons into an 8 bit input for the input/xmit state machine
unsigned char get_input_byte()
{
    unsigned char input;
    input = 0;

    if (SW1_STATE.state == BUTTON_DOWN)
        input |= SW1_DOWN;
    else
        input |= SW1_UP;


    return input;
}

// returns state machine input for the receive/display state machine
unsigned char get_display_byte()
{
    unsigned char input;
    input = 0;

    if (last_displayed == DOT)
        input |= DISP_DOT;
    else
        input |= DISP_DASH;

    if (disp_buffer.bit < disp_buffer.n_bits)
        input |= DISP_RDY;
    else
        input |= DISP_COMPLETE;

    return input;
}


// ===============
// transmit code
// ================
enum TX_STATE {
    TX_IDLE,
    TX_START,
    TX_BUSY
};

static volatile enum TX_STATE transmit_state = TX_IDLE;

void enable_xmit()
{
    while(TIMSK & _BV(TOIE0));
    transmit_state = TX_START;
    TIMSK |= _BV(TOIE0) | _BV(OCIE0);
}

void disable_xmit()
{
    TIMSK &= ~(_BV(TOIE0) | _BV(OCIE0));
    TCNT0 = 0;
}

uint8_t overflows = 0;
ISR (TIMER0_OVF_vect) {

    // only send a bit every overflow
    overflows++;
    if (overflows < 2)
        return;

    overflows = 0;

    uint8_t duty = DUTY_IDLE;
    char bit;
    switch(transmit_state) {
        case TX_START:
            duty = DUTY_START;
            transmit_state = TX_BUSY;
            break;
        case TX_BUSY:
            bit = buffer_pop_bit(&tx_buffer);
            if (bit == DOT)
                duty = DUTY_DOT;
            else if (bit == DASH)
                duty = DUTY_DASH;
            else {
                duty = DUTY_STOP;
                transmit_state = TX_IDLE;
            }
            break;
        case TX_IDLE:
            disable_xmit();
            duty = DUTY_IDLE;
            break;
    }

    OCR0 = duty;

    if (duty > 0)
        TX_PORT &= ~TX_PIN;
    else
        TX_PORT |= TX_PIN;
}

ISR (TIMER0_COMP_vect) {
    TX_PORT |= TX_PIN;
}

void setup_timer0() {
    TCNT0 =	0;
    TCCR0 = _BV(CS00) | _BV(CS02); //1024x prescaler
}

// =============
// receive code
// =============
enum RX_STATE {
    RX_IDLE,
    RX_BUSY,
    RX_COMPLETE
};
volatile static enum RX_STATE receive_state = RX_IDLE;
volatile static unsigned long t2_overflows = 0;

void start_timer2()
{
    TCNT2 = 0;
    TCCR2 = _BV(CS22) | _BV(CS21) | _BV(CS20);
    TIMSK |= _BV(TOIE2);
}


ISR(TIMER2_OVF_vect)
{
    t2_overflows++;
}

int duty_matches(uint16_t input, uint16_t wanted)
{
    if (input <= wanted + DUTY_TOLERANCE && input >= wanted - DUTY_TOLERANCE)
        return 1;
    else
        return 0;
}

uint32_t timer2_ticks()
{
    // based on a1 q2 sample sol'n
    uint32_t ticks;
    TIMSK &= ~_BV(TOIE2);
    ticks = TCNT2;
    // NOTE: must allow for overflow occurring between the read and our check
    if ( (TIFR & _BV(TOV2)) == _BV(TOV2) && ticks < 0x00ff )
        ticks += ((t2_overflows+1)*0xFFFF);
    else
        ticks += (t2_overflows*0xFFFF);
    TIMSK |= _BV(TOIE2);
    return ticks;
}

uint32_t pulse_start = 0;
ISR(INT0_vect) {
    uint32_t diff;
    uint8_t duty;
    //PORTB |= _BV(PD6) | _BV(PD5);
    if (RX_PORT & RX_PIN) {
        diff = timer2_ticks() - pulse_start;
        if (diff < 0xFF)
        {
            duty = diff & 0xFF;

            switch(receive_state) {
                case RX_IDLE:
                    if (duty_matches(duty, DUTY_START)) {
                        receive_state = RX_BUSY;
                    }
                    break;

                case RX_BUSY:
                    if (duty_matches(duty, DUTY_STOP)) {
                        receive_state = RX_COMPLETE;
                    } else if (duty_matches(duty, DUTY_DOT)) {
                        buffer_push_bit(&rx_buffer, DOT);
                    } else if (duty_matches(duty, DUTY_DASH)) {
                        buffer_push_bit(&rx_buffer, DASH);
                    } else {
                        receive_state = RX_IDLE;
                    }

                    break;
                case RX_COMPLETE:
                    break;
            }


        }
    } else {
        pulse_start = timer2_ticks();
    }

    /*
       PORTB |= 0xF0;
       switch(receive_state)
       {
       case RX_IDLE:
       PORTB &= ~_BV(PB7);
       break;
       case RX_BUSY:
       PORTB &= ~_BV(PB6);
       break;
       case RX_COMPLETE:
       PORTB &= ~_BV(PB5);
       }
       */
}

void enable_rx()
{
    // enable interrupt on either edge of receive line
    MCUCR |= _BV(ISC00);
    GICR |= _BV(INT0);
    start_timer2();
}

// ==============
// Main function
// ==============
int main(int argc, char **argv) {

    uint32_t clock;

    LED_DIR = 0xff; /* set led port to output */
    LED_PORT = 0xFF; // set LEDs to off
    SWITCH_DIR = 0x00;

    TX_DIR |= TX_PIN;
    RX_DIR &= ~RX_PIN;

    disable_jtag();
    setup_timer0();
    setup_timer1();

    enable_rx();

    // globally enable interrupts
    sei();

    int turn = 0;
    for (;;) {
        // snapshot clock as interrupt is continuously updating it
        clock = elapsed_ms;

        if (turn == TURN_INPUT_TX)
        {
            // update button state
            hysterisis_poll(SWITCH_PIN, &SW1_STATE);

            // input complete, start transmit
            if (input_state_machine.state == TRANSMIT) {
                // must wait for xmit to be ready
                if (transmit_state == TX_IDLE) {
                    // start transmit
                    buff_set(&tx_buffer, &input_buffer);
                    enable_xmit();

                    // re-enable input
                    buff_reset(&input_buffer);
                    input_state_machine.state = INPUT_IDLE;
                }
            }

            update_state_machine(&input_state_machine, clock, get_input_byte());
        } else {

            // check if receive has completed
            if (receive_state == RX_COMPLETE)
            {
                // wait for display to be idle
                if (display_state_machine.state == DISP_IDLE) {
                    // start the display
                    buff_set(&disp_buffer, &rx_buffer);
                    display_state_machine.state = DISPLAY_START;

                    // re-enable receive
                    receive_state = RX_IDLE;
                    buff_reset(&rx_buffer);
                }
            }

            update_state_machine(&display_state_machine, clock, get_display_byte());
        }

        turn = !turn;

    }
    return 0;
}
