//******************ECE 132*************************
// Names: Cesar Islas and Hardhik Mandadi
// Lab section: Tuesday 1:35-4:15 PM
//*************************************************
// Date Started: 4/21/26
// Date of Last Modification: 4/21/26
// Lab Assignment: Project 2 - FSM UART Test
//*************************************************
// Purpose of program: Minimal FSM test using only UART output and two switches.
// Used to verify that all 5 FSM states and their transitions work correctly
// before integrating IR sensor, ADC, servo, encoder, and watchdog.
//
// This file intentionally excludes: ADC, servo, encoder, watchdog, IR sensor.
//
// Program Inputs:
//   SW1 (PF4) - Simulates button press (start set, restart)
//   SW2 (PF0) - Simulates rep detection (stand-in for IR sensor)
//
// Program Outputs:
//   UART0 (PA0/PA1) - Prints current state, transitions, and rep/set counts
//
// How to test each transition:
//   SETUP          -> press SW1           -> should print WORKOUT_ACTIVE
//   WORKOUT_ACTIVE -> press SW2 N times   -> each press prints rep count
//                  -> after targetReps SW2 presses -> prints SET_COMPLETE then REST_PERIOD
//   REST_PERIOD    -> press SW1           -> simulates rest timer expiring -> WORKOUT_ACTIVE
//                  -> after targetSets sets -> prints WORKOUT_DONE
//   WORKOUT_DONE   -> press SW1           -> prints SETUP
//*************************************************

// File Include Statements
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

// State Index Definitions
#define SETUP_STATE          0
#define WORKOUT_ACTIVE_STATE 1
#define SET_COMPLETE_STATE   2
#define REST_PERIOD_STATE    3
#define WORKOUT_DONE_STATE   4

// Input Index Definitions
// next[0] = no input
// next[1] = SW1 pressed  (button — start/confirm/restart)
// next[2] = SW2 pressed  (rep sim — stand-in for IR sensor)
#define IN_NONE              0
#define IN_SW1               1
#define IN_SW2               2

// FSM State Struct
// green_out : green LED state (1=on, 0=off) - for future hardware use
// red_out   : red LED state   (1=on, 0=off) - for future hardware use
// wait      : delay in ms to hold state before sampling input (like T in course example)
// next[3]   : next state for IN_NONE, IN_SW1, IN_SW2
struct state {
    int green_out;
    int red_out;
    unsigned long wait;
    int next[3];
};

typedef struct state stype;

// FSM Table (5 states x 3 inputs)
//
// Reading a row:
// { green, red, wait_ms, {IN_NONE, IN_SW1, IN_SW2} }
//
// For this test file:
//   SW1 = button  (start, confirm rest done, restart)
//   SW2 = rep sim (each press = one rep, stand-in for IR)
//
// Transitions:
//   SETUP          + SW1 -> WORKOUT_ACTIVE
//   WORKOUT_ACTIVE + SW2 -> stays (rep counted), or SET_COMPLETE if reps done
//   SET_COMPLETE         -> auto-transitions to REST_PERIOD after wait
//   REST_PERIOD    + SW1 -> WORKOUT_ACTIVE (or WORKOUT_DONE if last set)
//   WORKOUT_DONE   + SW1 -> SETUP
stype fsm[5] = {
/* SETUP          { green, red, wait_ms, {IN_NONE, IN_SW1, IN_SW2} } */
{ 0, 1, 100,
  { SETUP_STATE, WORKOUT_ACTIVE_STATE, SETUP_STATE } },

/* WORKOUT_ACTIVE */
{ 1, 0, 0,
  { WORKOUT_ACTIVE_STATE, WORKOUT_ACTIVE_STATE, WORKOUT_ACTIVE_STATE } },

/* SET_COMPLETE */
{ 0, 1, 500,
  { REST_PERIOD_STATE, REST_PERIOD_STATE, REST_PERIOD_STATE } },

/* REST_PERIOD */
{ 0, 1, 0,
  { REST_PERIOD_STATE, WORKOUT_ACTIVE_STATE, REST_PERIOD_STATE } },

/* WORKOUT_DONE */
{ 0, 1, 200,
  { WORKOUT_DONE_STATE, SETUP_STATE, WORKOUT_DONE_STATE } }
};

// Global Variables
int currentState  = SETUP_STATE;
int repCount      = 0;
int setsCompleted = 0;
int targetReps    = 3; // Small number for easy testing
int targetSets    = 2; // Small number for easy testing

// UART Strings
char msg_welcome[]  = "\r\n=== FSM UART TEST ===\r\n";
char msg_targets[]  = "Target Reps: ";
char msg_tsets[]    = "  Target Sets: ";
char msg_howto[]    = "SW1 = button | SW2 = rep (IR stand-in)\r\n";
char msg_divider[]  = "------------------------------\r\n";
char msg_setup[]    = "STATE: SETUP - Press SW1 to begin workout\r\n";
char msg_active[]   = "STATE: WORKOUT_ACTIVE\r\n";
char msg_setdone[]  = "STATE: SET_COMPLETE\r\n";
char msg_rest[]     = "STATE: REST_PERIOD - Press SW1 to end rest\r\n";
char msg_wrkdone[]  = "STATE: WORKOUT_DONE - Press SW1 to restart\r\n";
char msg_rep[]      = "  Rep: ";
char msg_slash[]    = "/";
char msg_set[]      = "  Set complete! Sets done: ";
char msg_newline[]  = "\r\n";
char msg_sw1[]      = "[SW1 pressed]\r\n";
char msg_sw2[]      = "[SW2 pressed]\r\n";
char msg_ignored[]  = "  (input ignored in this state)\r\n";

// Function Prototypes
void switchISR(void);
void printString(char *str);
void printInt(int val);
void printState(int state);
void portF_output_setup(uint8_t pin);
void portF_input_setup(uint8_t pin);
void uart_setup(void);

// MAIN
int main(void) {

    // System clock: 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Setup UART and GPIO only — no ADC, servo, encoder, or watchdog
    uart_setup();
    portF_output_setup(0x0E); // PF1 (red), PF2, PF3 (green) — outputs for future use
    portF_input_setup(0x11);  // PF0 (SW2) + PF4 (SW1) inputs

    // GPIO Interrupts: SW1 (PF4) and SW2 (PF0), falling edge (active low)
    GPIOIntTypeSet(GPIO_PORTF_BASE, 0x11, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, switchISR);
    GPIOIntEnable(GPIO_PORTF_BASE, 0x11);

    IntMasterEnable();

    // Print startup banner
    printString(msg_welcome);
    printString(msg_targets);
    printInt(targetReps);
    printString(msg_tsets);
    printInt(targetSets);
    printString(msg_newline);
    printString(msg_howto);
    printString(msg_divider);

    // Initialize to SETUP
    currentState = SETUP_STATE;
    printState(currentState);

    // Main Loop
    // FSM step 6: output -> wait -> (input via ISR) -> next state
    // Inputs are handled entirely in switchISR.
    // Main loop applies outputs and executes the wait delay.
    while (1) {

        // 1. Apply LED outputs for current state (Moore: output = f(state))
        //    LEDs not wired in this test but outputs set for future integration
        if (fsm[currentState].green_out) { GPIO_PORTF_DATA_R |=  0x08; }
        else                             { GPIO_PORTF_DATA_R &= ~0x08; }
        if (fsm[currentState].red_out)   { GPIO_PORTF_DATA_R |=  0x02; }
        else                             { GPIO_PORTF_DATA_R &= ~0x02; }

        // 2. Wait — hold this state for wait ms (T field from FSM example)
        //    SysCtlDelay(n) = n * 3 clock cycles at 50 MHz
        unsigned long waitCycles = fsm[currentState].wait * (SysCtlClockGet() / 3000);
        if (waitCycles > 0) {
            SysCtlDelay(waitCycles);
        }

        // 3. Input and state update handled in switchISR
        //    (ISR fires on SW1/SW2 press and updates currentState)

        // 4. Auto-transition: SET_COMPLETE moves to REST_PERIOD after its wait
        if (currentState == SET_COMPLETE_STATE) {
            currentState = REST_PERIOD_STATE;
            printString(msg_rest);
        }
    }
}

// Switch ISR — handles SW1 (PF4) and SW2 (PF0)
//
// SW1 = button:  starts workout, ends rest, restarts after done
// SW2 = rep sim: each press counts as one rep (IR stand-in)
//
// FSM step 4: currentState = fsm[currentState].next[input]
void switchISR(void) {

    // Read which switch was pressed (active low)
    int sw1 = (GPIO_PORTF_DATA_R & 0x10) ? 0 : 1;
    int sw2 = (GPIO_PORTF_DATA_R & 0x01) ? 0 : 1;

    // Determine input index
    int input;
    if (sw1)      { input = IN_SW1; printString(msg_sw1); }
    else if (sw2) { input = IN_SW2; printString(msg_sw2); }
    else          { input = IN_NONE; }

    // --- Handle SW2 (rep simulation) during WORKOUT_ACTIVE ---
    if (input == IN_SW2 && currentState == WORKOUT_ACTIVE_STATE) {
        repCount++;
        printString(msg_rep);
        printInt(repCount);
        printString(msg_slash);
        printInt(targetReps);
        printString(msg_newline);

        // Check if target reps reached
        if (repCount >= targetReps) {
            repCount = 0;
            setsCompleted++;
            printString(msg_set);
            printInt(setsCompleted);
            printString(msg_slash);
            printInt(targetSets);
            printString(msg_newline);

            // Transition to WORKOUT_DONE if all sets complete, else SET_COMPLETE
            if (setsCompleted >= targetSets) {
                currentState = WORKOUT_DONE_STATE;
                printState(currentState);
            } else {
                currentState = SET_COMPLETE_STATE;
                printState(currentState);
                // Main loop will auto-advance SET_COMPLETE -> REST_PERIOD after wait
            }
        }
        // Re-arm and return — rep handling is complete
        GPIOIntClear(GPIO_PORTF_BASE, 0x11);
        return;
    }

    // --- Handle SW1 (button) and SW2 in all other states via FSM table ---
    if (input == IN_SW2 && currentState != WORKOUT_ACTIVE_STATE) {
        printString(msg_ignored);
        GPIOIntClear(GPIO_PORTF_BASE, 0x11);
        return;
    }

    // Look up next state from FSM table
    int nextState = fsm[currentState].next[input];

    // Only update and print if state actually changed
    if (nextState != currentState) {
        currentState = nextState;
        printState(currentState);
    }

    // Re-arm interrupt
    GPIOIntClear(GPIO_PORTF_BASE, 0x11);
}

// printState — prints the current state name to UART
void printState(int state) {
    printString(msg_divider);
    if (state == SETUP_STATE)          { printString(msg_setup);   }
    else if (state == WORKOUT_ACTIVE_STATE) { printString(msg_active);  }
    else if (state == SET_COMPLETE_STATE)   { printString(msg_setdone); }
    else if (state == REST_PERIOD_STATE)    { printString(msg_rest);    }
    else if (state == WORKOUT_DONE_STATE)   { printString(msg_wrkdone); }
}

// Peripheral Setup Functions

void uart_setup(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

void portF_output_setup(uint8_t pin) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIO_PORTF_DIR_R |= pin;
    GPIO_PORTF_DEN_R |= pin;
}

void portF_input_setup(uint8_t pin) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock (required for PF0)
    GPIO_PORTF_CR_R  |= pin;
    GPIO_PORTF_DIR_R &= ~pin;
    GPIO_PORTF_PUR_R |= pin;        // Pull-up resistors (active low)
    GPIO_PORTF_DEN_R |= pin;
}

// printString / printInt — UART output helpers
void printString(char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str++);
    }
}

void printInt(int val) {
    if (val == 0) { UARTCharPut(UART0_BASE, '0'); return; }
    char buf[12];
    int i = 0;
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    for (int j = i - 1; j >= 0; j--) {
        UARTCharPut(UART0_BASE, buf[j]);
    }
}
