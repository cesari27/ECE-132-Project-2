//******************ECE 132*************************
// Names: Cesar Islas and Hardhik Mandadi
// Lab section: Tuesday 1:35-4:15 PM
//*************************************************
// Date Started: 4/21/26
// Date of Last Modification: 4/21/26
// Note: No software debouncing used. All inputs handled via hardware interrupts.
// Lab Assignment: Project 2
//*************************************************
// Purpose of program: Moore FSM workout rep counter and set tracker.
// The system traverses through 5 states driven by an IR sensor (rep detection),
// push button (set start/confirm), rotary encoder (set reps/sets), potentiometer
// (rest time via ADC), servo motor (sets remaining indicator), and LEDs (status).
//
// Program Inputs:
//   IR Sensor    (PE4)         - Detects reps during WORKOUT_ACTIVE
//   Push Button  (PF4 = SW1)   - Start set, confirm, reset
//   Rotary Enc A (PB0)         - Encoder channel A
//   Rotary Enc B (PB1)         - Encoder channel B
//   Potentiometer(PE3 / AIN0)  - ADC input for rest time duration
//
// Program Outputs:
//   Green LED    (PF3)         - Active/running indicator
//   Red LED      (PF1)         - Inactive/rest/done indicator
//   Servo Motor  (PB6 / M0PWM0)- Needle showing sets remaining
//   UART0        (PA0/PA1)     - Debug and status messages
//*************************************************

// -------------------------------------------------------
// File Include Statements
// -------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/watchdog.h"

// -------------------------------------------------------
// State Index Definitions
// Used in fsm[] array struct
// -------------------------------------------------------
#define SETUP_STATE          0
#define WORKOUT_ACTIVE_STATE 1
#define SET_COMPLETE_STATE   2
#define REST_PERIOD_STATE    3
#define WORKOUT_DONE_STATE   4

// -------------------------------------------------------
// Input Index Definitions
// next[0] = no input
// next[1] = button pressed
// next[2] = reps reached target
// next[3] = rest timer expired
// next[4] = all sets done
// -------------------------------------------------------
#define IN_NONE              0
#define IN_BUTTON            1
#define IN_REPS_DONE         2
#define IN_REST_DONE         3
#define IN_ALL_SETS_DONE     4

// -------------------------------------------------------
// Hardware Pin Definitions
// -------------------------------------------------------
#define GREEN_LED_PIN        GPIO_PIN_3   // PF3
#define RED_LED_PIN          GPIO_PIN_1   // PF1
#define BUTTON_PIN           GPIO_PIN_4   // PF4 (SW1)
#define IR_SENSOR_PIN        GPIO_PIN_4   // PE4
#define ENC_A_PIN            GPIO_PIN_0   // PB0
#define ENC_B_PIN            GPIO_PIN_1   // PB1

// -------------------------------------------------------
// Servo PWM Constants (50 Hz, 20 ms period)
// Tiva PWM clock = SysClk / 64 = ~781 kHz at 50 MHz
// Period counts = 781000 / 50 = 15620
// Min pulse ~1 ms = 781 counts, Max ~2 ms = 1562 counts
// -------------------------------------------------------
#define PWM_PERIOD           15620
#define SERVO_MIN            781
#define SERVO_MAX            1562

// -------------------------------------------------------
// Watchdog Timeout Constant
// At 50 MHz: 1 tick = 20 ns, 50,000,000 ticks = 1 second
// WDT_RELOAD = 50,000,000 * 30 = ~30 seconds
// If the main loop stops feeding the watchdog for 30 seconds
// (e.g., user walked away mid-rest), the system resets to WORKOUT_DONE.
// Adjust the multiplier to tune the grace period.
// -------------------------------------------------------
#define WDT_RELOAD           (50000000UL * 30) // ~30 second watchdog timeout at 50 MHz

// -------------------------------------------------------
// FSM State Struct
// green_out  : state of green LED (1=on, 0=off)
// red_out    : state of red LED   (1=on, 0=off)
// ir_active  : whether IR sensor counts reps in this state
// wait       : delay in ms to hold this state before sampling input
//              Mirrors the T (time delay) field from the course FSM example.
//              Gives the system time to stabilize outputs before transitioning.
// next[5]    : next state for each input index
// -------------------------------------------------------
struct state {
    int green_out;
    int red_out;
    int ir_active;
    unsigned long wait;  // delay in ms for this state (like T in course example)
    int next[5]; // indexed by IN_NONE, IN_BUTTON, IN_REPS_DONE, IN_REST_DONE, IN_ALL_SETS_DONE
};

typedef struct state stype;

// -------------------------------------------------------
// FSM Table (5 states x 5 inputs)
//
// Reading a row:
// { green, red, ir_active, wait_ms, {IN_NONE, IN_BUTTON, IN_REPS_DONE, IN_REST_DONE, IN_ALL_SETS_DONE} }
//
// wait_ms: how long (in ms) to hold this state before sampling inputs.
//   SETUP          = 100 ms  - short poll delay while user configures via encoder
//   WORKOUT_ACTIVE = 0 ms    - no delay; IR interrupt drives transitions instantly
//   SET_COMPLETE   = 500 ms  - brief pause so LEDs are visible before moving to REST
//   REST_PERIOD    = 0 ms    - rest duration is controlled by ADC timer, not wait field
//   WORKOUT_DONE   = 200 ms  - short delay to prevent accidental immediate restart
//
// SETUP          -> button press starts WORKOUT_ACTIVE
// WORKOUT_ACTIVE -> reps hit target => SET_COMPLETE (or WORKOUT_DONE if last set)
// SET_COMPLETE   -> transitions immediately to REST_PERIOD after wait
// REST_PERIOD    -> rest timer expires => WORKOUT_ACTIVE (or WORKOUT_DONE if last set)
// WORKOUT_DONE   -> button press resets to SETUP
// -------------------------------------------------------
stype fsm[5] = {
/* SETUP          { green, red, ir_active, wait_ms, {next states} } */
{ 0, 1, 0, 100,
  { SETUP_STATE, WORKOUT_ACTIVE_STATE, SETUP_STATE, SETUP_STATE, SETUP_STATE } },

/* WORKOUT_ACTIVE */
{ 1, 0, 1, 0,
  { WORKOUT_ACTIVE_STATE, WORKOUT_ACTIVE_STATE, SET_COMPLETE_STATE, WORKOUT_ACTIVE_STATE, WORKOUT_DONE_STATE } },

/* SET_COMPLETE */
{ 0, 1, 0, 500,
  { REST_PERIOD_STATE, REST_PERIOD_STATE, REST_PERIOD_STATE, REST_PERIOD_STATE, WORKOUT_DONE_STATE } },

/* REST_PERIOD */
{ 0, 1, 0, 0,
  { REST_PERIOD_STATE, REST_PERIOD_STATE, REST_PERIOD_STATE, WORKOUT_ACTIVE_STATE, WORKOUT_DONE_STATE } },

/* WORKOUT_DONE */
{ 0, 1, 0, 200,
  { WORKOUT_DONE_STATE, SETUP_STATE, WORKOUT_DONE_STATE, WORKOUT_DONE_STATE, WORKOUT_DONE_STATE } }
};

// -------------------------------------------------------
// Global State Variables
// -------------------------------------------------------
volatile int  currentState    = SETUP_STATE;
volatile int  pendingInput    = IN_NONE;     // Set by ISR, consumed by main loop

// Workout tracking
volatile int  repCount        = 0;           // Reps completed in current set
volatile int  setsCompleted   = 0;           // Sets finished so far
int           targetReps      = 10;          // Configurable via rotary encoder
int           targetSets      = 3;           // Configurable via rotary encoder

// Rotary encoder: tracks which parameter is being edited in SETUP
// 0 = editing targetReps, 1 = editing targetSets
int           encoderMode     = 0;

// System tick counter in ms — driven by Timer0A ISR (used for rest timer)
volatile uint32_t sysTick_ms  = 0;

// Rest timer (counts down in ms, loaded from ADC/potentiometer reading)
volatile uint32_t restTimer_ms    = 0;
volatile int      restTimerActive = 0;

// Watchdog feed flag — main loop sets this every iteration
volatile int wdtFed = 0;

// -------------------------------------------------------
// UART Strings
// -------------------------------------------------------
char msg_welcome[]    = "\r\n=== Workout Rep Counter Started ===\r\n";
char msg_setup[]      = "STATE: SETUP - Use encoder to set reps/sets, press button to begin\r\n";
char msg_active[]     = "STATE: WORKOUT_ACTIVE\r\n";
char msg_setdone[]    = "STATE: SET_COMPLETE\r\n";
char msg_rest[]       = "STATE: REST_PERIOD\r\n";
char msg_wrkdone[]    = "STATE: WORKOUT_DONE - Press button to restart\r\n";
char msg_rep[]        = "Rep counted! Reps: ";
char msg_slash[]      = "/";
char msg_newline[]    = "\r\n";
char msg_sets[]       = "Sets completed: ";
char msg_rest_rem[]   = "Rest time remaining (ms): ";
char msg_wdt[]        = "WATCHDOG RESET - User walked away.\r\n";
char msg_config[]     = "CONFIG - Target Reps: ";
char msg_sets_cfg[]   = "  Target Sets: ";

// -------------------------------------------------------
// Function Prototypes
// -------------------------------------------------------
void IR_Sensor_ISR(void);
void Timer0A_ISR(void);
void applyOutputs(void);
void updateServo(int setsRemaining, int totalSets);
void printString(char *str);
void printInt(int val);
void printModeUART(int state);
void portF_output_setup(uint8_t pin);
void portF_input_setup(uint8_t pin);
void portE_input_setup(uint8_t pin);
void portB_setup(void);
void uart_setup(void);
void adc_setup(void);
void pwm_servo_setup(void);
void timer0_setup(void);
void watchdog_setup(void);
int  readButton(void);
int  readIR(void);
int  readEncoderDelta(void);
uint32_t readADC_restTime(void);
void feedWatchdog(void);

// -------------------------------------------------------
// MAIN
// -------------------------------------------------------
int main(void) {

    // System clock: 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Peripheral setup
    uart_setup();
    portF_output_setup(GREEN_LED_PIN | RED_LED_PIN); // PF1, PF3 outputs
    portF_input_setup(BUTTON_PIN);                   // PF4 input (SW1)
    portE_input_setup(IR_SENSOR_PIN);                // PE4 input (IR)
    portB_setup();                                   // PB0/PB1 encoder, PB6 servo PWM
    adc_setup();                                     // PE3 potentiometer
    pwm_servo_setup();                               // PB6 servo
    timer0_setup();                                  // 1 ms system tick + rest timer
    watchdog_setup();                                // ~10 second timeout

    // GPIO interrupt: IR sensor on PE4, falling edge (active low = object detected)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOIntTypeSet(GPIO_PORTE_BASE, IR_SENSOR_PIN, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTE_BASE, IR_Sensor_ISR);
    GPIOIntEnable(GPIO_PORTE_BASE, IR_SENSOR_PIN);

    IntMasterEnable();

    // Print startup message
    printString(msg_welcome);
    currentState = SETUP_STATE;
    printModeUART(currentState);
    printString(msg_setup);

    // -------------------------------------------------------
    // Main Loop - FSM driven, inputs sampled each iteration
    // -------------------------------------------------------
    while (1) {

        // --- Feed watchdog each loop iteration ---
        // If the loop stalls (user walked away, system hung),
        // the WDT fires and resets to WORKOUT_DONE
        feedWatchdog();

        // --- Sample rotary encoder in SETUP to configure reps/sets ---
        if (currentState == SETUP_STATE) {
            int delta = readEncoderDelta();
            if (delta != 0) {
                if (encoderMode == 0) {
                    targetReps += delta;
                    if (targetReps < 1)  targetReps = 1;
                    if (targetReps > 30) targetReps = 30;
                } else {
                    targetSets += delta;
                    if (targetSets < 1)  targetSets = 1;
                    if (targetSets > 10) targetSets = 10;
                }
                // Print current config to UART
                printString(msg_config);
                printInt(targetReps);
                printString(msg_sets_cfg);
                printInt(targetSets);
                printString(msg_newline);
            }
        }

        // --- Determine FSM input for this cycle ---
        int input = IN_NONE;

        // Button press: used in SETUP (start), WORKOUT_DONE (restart)
        if (readButton()) {
            input = IN_BUTTON;
        }

        // IR interrupt set the pendingInput flag (reps done)
        if (pendingInput == IN_REPS_DONE) {
            if (setsCompleted + 1 >= targetSets) {
                input = IN_ALL_SETS_DONE; // Last set just finished
            } else {
                input = IN_REPS_DONE;
            }
            pendingInput = IN_NONE;
        }

        // Rest timer expired
        if (currentState == REST_PERIOD_STATE && restTimerActive && restTimer_ms == 0) {
            restTimerActive = 0;
            if (setsCompleted >= targetSets) {
                input = IN_ALL_SETS_DONE;
            } else {
                input = IN_REST_DONE;
            }
        }

        // All sets done check (catch-all)
        if (setsCompleted >= targetSets && currentState == WORKOUT_ACTIVE_STATE) {
            input = IN_ALL_SETS_DONE;
        }

        // --- FSM Transition ---
        int nextState = fsm[currentState].next[input];

        if (nextState != currentState) {

            // State entry actions
            if (nextState == WORKOUT_ACTIVE_STATE) {
                repCount = 0; // Reset rep count for new set
                printString(msg_active);
                printString(msg_sets);
                printInt(setsCompleted);
                printString(msg_slash);
                printInt(targetSets);
                printString(msg_newline);
            }

            if (nextState == SET_COMPLETE_STATE) {
                setsCompleted++;
                printString(msg_setdone);
                printString(msg_sets);
                printInt(setsCompleted);
                printString(msg_slash);
                printInt(targetSets);
                printString(msg_newline);

                // Load rest time from potentiometer (ADC)
                restTimer_ms    = readADC_restTime();
                restTimerActive = 1;

                printString(msg_rest_rem);
                printInt((int)restTimer_ms);
                printString(msg_newline);
            }

            if (nextState == REST_PERIOD_STATE) {
                printString(msg_rest);
            }

            if (nextState == WORKOUT_DONE_STATE) {
                printString(msg_wrkdone);
                restTimerActive = 0;
                restTimer_ms    = 0;
            }

            if (nextState == SETUP_STATE) {
                // Full reset for new session
                repCount       = 0;
                setsCompleted  = 0;
                encoderMode    = 0;
                printModeUART(nextState);
                printString(msg_setup);
            }

            currentState = nextState;
        }

        // SET_COMPLETE immediately transitions to REST_PERIOD (Moore: output only depends on state)
        if (currentState == SET_COMPLETE_STATE) {
            currentState = REST_PERIOD_STATE;
            printString(msg_rest);
        }

        // --- Apply outputs based on current state (Moore FSM) ---
        applyOutputs();

        // --- FSM Wait (T field) ---
        // After applying outputs, hold this state for wait ms before sampling inputs.
        // This mirrors the T delay field from the course FSM example:
        //   cstate = fsm[cstate.next[input]] after the wait expires.
        // WORKOUT_ACTIVE and REST_PERIOD have wait=0 so they respond instantly.
        // SysCtlDelay(n) loops for n*3 clock cycles at 50 MHz.
        // To convert wait_ms to SysCtlDelay counts: (50,000,000 / 3000) * wait_ms
        unsigned long waitCycles = (fsm[currentState].wait * (SysCtlClockGet() / 3000));
        if (waitCycles > 0) {
            SysCtlDelay(waitCycles);
        }

        // --- Update servo: needle points to sets remaining ---
        int setsRemaining = targetSets - setsCompleted;
        if (setsRemaining < 0) setsRemaining = 0;
        updateServo(setsRemaining, targetSets);
    }
}

// -------------------------------------------------------
// IR Sensor Interrupt Handler (PE4, falling edge)
//
// Fires when IR beam is broken = one rep detected.
// No software debouncing — interrupt hardware handles edge detection.
// Gates on WORKOUT_ACTIVE state, increments repCount, and sets
// pendingInput flag if target reps reached.
// Heavy logic (FSM transitions, UART) stays in main loop.
// -------------------------------------------------------
void IR_Sensor_ISR(void) {

    // 1. Clear interrupt flag immediately to prevent re-triggering
    GPIOIntClear(GPIO_PORTE_BASE, IR_SENSOR_PIN);

    // 2. State gate: only count reps during WORKOUT_ACTIVE
    if (currentState != WORKOUT_ACTIVE_STATE) {
        return;
    }

    // 3. Only count if IR input is enabled for this state
    if (!fsm[currentState].ir_active) {
        return;
    }

    // 4. Increment rep counter and print to UART
    repCount++;
    printString(msg_rep);
    printInt(repCount);
    printString(msg_slash);
    printInt(targetReps);
    printString(msg_newline);

    // 5. Brief green LED toggle for per-rep visual feedback
    GPIO_PORTF_DATA_R ^= GREEN_LED_PIN;

    // 6. Check if target reps reached — signal main loop via flag
    //    Main loop handles the actual FSM transition (keeps ISR short)
    if (repCount >= targetReps) {
        repCount     = 0;
        pendingInput = IN_REPS_DONE; // Main loop reads and clears this
    }
}

// -------------------------------------------------------
// Timer0A ISR — fires every 1 ms
// Maintains sysTick_ms counter and counts down restTimer_ms
// -------------------------------------------------------
void Timer0A_ISR(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    sysTick_ms++;

    // Count down rest timer if active
    if (restTimerActive && restTimer_ms > 0) {
        restTimer_ms--;
    }
}

// -------------------------------------------------------
// applyOutputs
// Drives hardware pins based on current FSM state outputs.
// Pure Moore: outputs depend ONLY on currentState.
// -------------------------------------------------------
void applyOutputs(void) {
    int g = fsm[currentState].green_out;
    int r = fsm[currentState].red_out;

    // Green LED: PF3
    if (g) { GPIO_PORTF_DATA_R |=  GREEN_LED_PIN; }
    else   { GPIO_PORTF_DATA_R &= ~GREEN_LED_PIN; }

    // Red LED: PF1
    if (r) { GPIO_PORTF_DATA_R |=  RED_LED_PIN; }
    else   { GPIO_PORTF_DATA_R &= ~RED_LED_PIN; }
}

// -------------------------------------------------------
// updateServo
// Maps setsRemaining to a PWM pulse width.
// Full sets = servo at max angle; 0 sets = servo at min.
// -------------------------------------------------------
void updateServo(int setsRemaining, int totalSets) {
    if (totalSets == 0) totalSets = 1; // avoid divide by zero
    uint32_t pulse = SERVO_MIN + ((uint32_t)(setsRemaining) * (SERVO_MAX - SERVO_MIN)) / totalSets;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulse);
}

// -------------------------------------------------------
// readButton - PF4, active low (returns 1 if pressed)
// -------------------------------------------------------
int readButton(void) {
    return (GPIO_PORTF_DATA_R & BUTTON_PIN) ? 0 : 1;
}

// -------------------------------------------------------
// readIR - PE4, active low (returns 1 if beam broken)
// -------------------------------------------------------
int readIR(void) {
    return (GPIO_PORTE_DATA_R & IR_SENSOR_PIN) ? 0 : 1;
}

// -------------------------------------------------------
// readEncoderDelta
// Reads rotary encoder on PB0 (A) and PB1 (B).
// Returns +1 for CW, -1 for CCW, 0 for no change.
// Simple single-edge detection on channel A.
// -------------------------------------------------------
int readEncoderDelta(void) {
    static int lastA = 1;
    int A = (GPIO_PORTB_DATA_R & ENC_A_PIN) ? 1 : 0;
    int B = (GPIO_PORTB_DATA_R & ENC_B_PIN) ? 1 : 0;
    int delta = 0;

    if (A != lastA) {       // Edge detected on A
        if (A == 0) {       // Falling edge
            delta = (B == 1) ? +1 : -1;
        }
        lastA = A;
    }
    return delta;
}

// -------------------------------------------------------
// readADC_restTime
// Reads potentiometer on PE3 (AIN0).
// Maps 0-4095 ADC reading to 5000-30000 ms rest window.
// Low voltage = shorter rest, high voltage = longer rest.
// -------------------------------------------------------
uint32_t readADC_restTime(void) {
    uint32_t adcVal[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, adcVal);

    // Map 0-4095 -> 5000-30000 ms
    uint32_t restMs = 5000 + (adcVal[0] * 25000UL / 4095);
    return restMs;
}

// -------------------------------------------------------
// feedWatchdog
// Resets the WDT countdown. Call every main loop iteration.
// If main loop stalls, WDT fires and resets the system.
// -------------------------------------------------------
void feedWatchdog(void) {
    WatchdogReloadSet(WATCHDOG0_BASE, WDT_RELOAD);
}

// -------------------------------------------------------
// Peripheral Setup Functions
// -------------------------------------------------------

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
    GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock PF0 if needed
    GPIO_PORTF_CR_R  |= pin;
    GPIO_PORTF_DIR_R &= ~pin;
    GPIO_PORTF_PUR_R |= pin;        // Pull-up (active low)
    GPIO_PORTF_DEN_R |= pin;
}

void portE_input_setup(uint8_t pin) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIO_PORTE_DIR_R &= ~pin;
    GPIO_PORTE_PUR_R |= pin;        // Pull-up (active low)
    GPIO_PORTE_DEN_R |= pin;
}

void portB_setup(void) {
    // PB0, PB1: rotary encoder inputs
    // PB6: servo PWM output (configured via PWM peripheral)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Encoder inputs
    GPIO_PORTB_DIR_R &= ~(ENC_A_PIN | ENC_B_PIN);
    GPIO_PORTB_PUR_R |=  (ENC_A_PIN | ENC_B_PIN);
    GPIO_PORTB_DEN_R |=  (ENC_A_PIN | ENC_B_PIN);
    // PB6 configured as PWM output inside pwm_servo_setup()
}

void adc_setup(void) {
    // ADC0, Sequencer 3 (single sample), PE3 = AIN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
        ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // AIN0 = PE3
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

void pwm_servo_setup(void) {
    // PWM0 Module, Generator 0, Output 0 on PB6
    // PWM clock = SysClk / 64
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, SERVO_MIN);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void timer0_setup(void) {
    // Timer0A: periodic 1 ms interrupt for sysTick and rest countdown
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000); // 1 ms
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_ISR);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void watchdog_setup(void) {
    // Watchdog0: ~30 second timeout
    // At 50 MHz: WDT_RELOAD = 50,000,000 * 30 = 1,500,000,000 ticks = ~30 seconds
    // If the main loop stops feeding the watchdog (user walked away),
    // the system resets and re-initializes to WORKOUT_DONE state.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    WatchdogReloadSet(WATCHDOG0_BASE, WDT_RELOAD);
    WatchdogResetEnable(WATCHDOG0_BASE);
    WatchdogEnable(WATCHDOG0_BASE);
}

// -------------------------------------------------------
// UART Utility Functions
// -------------------------------------------------------

void printString(char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str++);
    }
}

void printInt(int val) {
    if (val < 0) {
        UARTCharPut(UART0_BASE, '-');
        val = -val;
    }
    if (val == 0) {
        UARTCharPut(UART0_BASE, '0');
        return;
    }
    char buf[12];
    int i = 0;
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    // Print in reverse (buf is backwards)
    for (int j = i - 1; j >= 0; j--) {
        UARTCharPut(UART0_BASE, buf[j]);
    }
}

void printModeUART(int state) {
    printString("Current State: ");
    if (state == SETUP_STATE)          { printString("SETUP\r\n"); }
    else if (state == WORKOUT_ACTIVE_STATE) { printString("WORKOUT_ACTIVE\r\n"); }
    else if (state == SET_COMPLETE_STATE)   { printString("SET_COMPLETE\r\n"); }
    else if (state == REST_PERIOD_STATE)    { printString("REST_PERIOD\r\n"); }
    else if (state == WORKOUT_DONE_STATE)   { printString("WORKOUT_DONE\r\n"); }
}
