// This is the configuration file. Ideally, any configuration and calibration
// of the device should be done by changing this file only.

// Needed by util/delay.h
#define F_CPU 1000000UL

// Delay between two executions of main loop, given in millisecond.

#define LOOP_DELAY 250

// Sets 1 MHz / 256 / 64 = 61 Hz pwm
#define PRESCALER_VALUE Atmega8::PSV_64

// Half length of one complete indicator period. In other words, the time the
// indicator spends on and off. Given in units of LOOP_DELAY.
#define INDICATOR_HALF_PERIOD 8

// Speeds and accelerations of motors, in range 0 .. 255 where 255 is the
// maximum.
#define MOTOR_1_SPEED 0xa0
#define MOTOR_2_INITIAL_SPEED 0x18
#define MOTOR_1_ACCELERATION 0xff
#define MOTOR_2_ACCELERATION 0x10

// Definition of indicator pin data direction and data ports and pins
#define INDICATOR_DATA_DIR DDRB
#define INDICATOR_DATA_DIR_PIN DDB0
#define INDICATOR_DATA PORTB
#define INDICATOR_DATA_PIN PORTB0

// Definitions of switch data in input ports and pins
#define SWITCH_1_DATA PORTD
#define SWITCH_1_DATA_PIN PORTD7
#define SWITCH_1_INPUT PIND
#define SWITCH_1_INPUT_PIN PIND7

#define SWITCH_2_DATA PORTB
#define SWITCH_2_DATA_PIN PORTB7
#define SWITCH_2_INPUT PINB
#define SWITCH_2_INPUT_PIN PINB7

// Definitions of switch data direction and data ports and pins
#define MOTOR_1_FORWARD_DATA_DIR DDRB
#define MOTOR_1_FORWARD_DATA_DIR_PIN DDB2
#define MOTOR_1_FORWARD_DATA PORTB
#define MOTOR_1_FORWARD_DATA_PIN PORTB2
#define MOTOR_1_FORWARD_OUTPUT_COMPARE OCR1BL

#define MOTOR_1_REVERSE_DATA_DIR DDRB
#define MOTOR_1_REVERSE_DATA_DIR_PIN DDB1
#define MOTOR_1_REVERSE_DATA PORTB
#define MOTOR_1_REVERSE_DATA_PIN PORTB1
#define MOTOR_1_REVERSE_OUTPUT_COMPARE OCR1AL

#define MOTOR_2_FORWARD_DATA_DIR DDRB
#define MOTOR_2_FORWARD_DATA_DIR_PIN DDB3
#define MOTOR_2_FORWARD_DATA PORTB
#define MOTOR_2_FORWARD_DATA_PIN PORTB3
#define MOTOR_2_FORWARD_OUTPUT_COMPARE OCR2A

#define MOTOR_2_REVERSE_DATA_DIR DDRD
#define MOTOR_2_REVERSE_DATA_DIR_PIN DDD3
#define MOTOR_2_REVERSE_DATA PORTD
#define MOTOR_2_REVERSE_DATA_PIN PORTD3
#define MOTOR_2_REVERSE_OUTPUT_COMPARE OCR2B
