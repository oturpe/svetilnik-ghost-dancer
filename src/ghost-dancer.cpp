// Controller for a dancing dress sculpture.
//
// Author: Otto Urpelainen
// Email: oturpe@iki.fi
// Date: 2016-06-18

#include "Atmega8Utils.h"

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

// {
//    // Forward speed to have in the end of sequence
//    forwardTarget,
//    // Reverse speed to have in the end of sequence
//    reverseTarget,
//    // Acceleration for speed change
//    acceleration,
//    // How long to retain speed after it has been reached
//    retain time,
//    // How long to move linearly after finding switch 1. Special codes:
//        0xff - keep moving indefinitely
//        0xfe - do not move
//    position
// }

#define KEEP_MOVING 0xff
#define HOLD 0xfe
#define DANCE_SEQUENCE_LENGTH 32
uint8_t danceSequence[DANCE_SEQUENCE_LENGTH][5] = {
    // Sequence 1: Linear and rotatary
    { 0x80, 0x00, 0x40, 1, KEEP_MOVING },
    { 0xff, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x40, 1, KEEP_MOVING },
    { 0x00, 0xff, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x40, 1, KEEP_MOVING },
    { 0xff, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x40, 1, KEEP_MOVING },
    { 0x00, 0xff, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x40, 1, KEEP_MOVING },
    { 0xff, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x40, 1, KEEP_MOVING },
    { 0x00, 0xff, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x40, 1, KEEP_MOVING },
    { 0xff, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x80, 0x00, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x40, 1, KEEP_MOVING },
    { 0x00, 0xff, 0x04, 1, KEEP_MOVING },
    { 0x00, 0x80, 0x04, 1, KEEP_MOVING },
    // Sequence 2: Just rotate
    { 0x00, 0x00, 0xf0, 80, 10},
    { 0x80, 0x00, 0x80, 1, HOLD },
    { 0xff, 0x00, 0x02, 5, HOLD },
    { 0x8f, 0x00, 0x02, 5, HOLD },
    { 0x00, 0x00, 0x02, 1, HOLD },
    { 0xff, 0x00, 0x02, 5, HOLD },
    { 0x8f, 0x00, 0x02, 5, HOLD },
    { 0x00, 0x00, 0x02, 1, HOLD }
};

/// \brief
///    Toggles the indicator led state.
void toggleIndicator() {
    static bool lit;

    if (lit) {
        INDICATOR_DATA &= ~BV(INDICATOR_DATA_PIN);
    }
    else {
        INDICATOR_DATA |= BV(INDICATOR_DATA_PIN);
    }

    lit = !lit;
}

/// \brief
///    Reads both switches and returns a boolean array telling if they are
///    pressed or not.
///
/// \param switchState
///    Output parameter. Array of two booleans whose value is set to true if
///    the corresponding switch is being pressed.
void readSwitches(bool * switchState) {
    switchState[0] = SWITCH_1_INPUT & BV(SWITCH_1_INPUT_PIN);
    switchState[1] = SWITCH_2_INPUT & BV(SWITCH_2_INPUT_PIN);
}

/// \brief
///    Sets the motors to the state they should have on startup.
void initializeMotors() {
    // Set running forwards
    MOTOR_1_FORWARD_OUTPUT_COMPARE = MOTOR_1_SPEED;
    MOTOR_1_REVERSE_OUTPUT_COMPARE = 0x00;

    MOTOR_2_FORWARD_OUTPUT_COMPARE = MOTOR_2_INITIAL_SPEED;
    MOTOR_2_REVERSE_OUTPUT_COMPARE = 0x00;

    // Enable all outputs
    MOTOR_1_FORWARD_DATA |= BV(MOTOR_1_FORWARD_DATA_PIN);
    MOTOR_2_FORWARD_DATA |= BV(MOTOR_2_FORWARD_DATA_PIN);
    MOTOR_1_REVERSE_DATA |= BV(MOTOR_1_REVERSE_DATA_PIN);
    MOTOR_2_REVERSE_DATA |= BV(MOTOR_2_REVERSE_DATA_PIN);
}

/// \brief
///    Calculates the correct acceleration to use when current and target speed
///    are as given.
int16_t acceleration(uint8_t current, uint8_t target, uint8_t maxAcceleration) {
    int16_t acceleration;

    acceleration = (int16_t)target - (int16_t)current;

    if (acceleration > maxAcceleration) {
        acceleration = maxAcceleration;
    }

    return acceleration;
}

/// \brief
///    Changes motor 1 speed according to given speed target and maximum
///    acceleration.
///
/// \param forwardTarget
///    Forward target speed
///
/// \param reverseTarget
///    Reverse target speed
///
/// \param acceleration
///    Desired acceleration
void updateMotor1Speed(
    uint8_t forwardTarget,
    uint8_t reverseTarget,
    uint8_t acceleration
) {
    if (!MOTOR_1_REVERSE_OUTPUT_COMPARE) {
        MOTOR_1_FORWARD_OUTPUT_COMPARE += ::acceleration(
            MOTOR_1_FORWARD_OUTPUT_COMPARE,
            forwardTarget,
            acceleration
        );
    }
    if (!MOTOR_1_FORWARD_OUTPUT_COMPARE) {
        MOTOR_1_REVERSE_OUTPUT_COMPARE += ::acceleration(
            MOTOR_1_REVERSE_OUTPUT_COMPARE,
            reverseTarget,
            acceleration
        );
    }
}

/// \brief
///    Changes motor 2 speed according to given speed target and maximum
///    acceleration.
///
/// See updateMotor1Speed() function documentation for details. This function
/// behaves identically to that one.
///
/// \return
///    If requested speed was achieved
bool updateMotor2Speed(
    uint8_t forwardTarget,
    uint8_t reverseTarget,
    uint8_t acceleration
) {
    if (!MOTOR_2_REVERSE_OUTPUT_COMPARE) {
        MOTOR_2_FORWARD_OUTPUT_COMPARE += ::acceleration(
            MOTOR_2_FORWARD_OUTPUT_COMPARE,
            forwardTarget,
            acceleration
        );
    }
    if (!MOTOR_2_FORWARD_OUTPUT_COMPARE) {
        MOTOR_2_REVERSE_OUTPUT_COMPARE += ::acceleration(
            MOTOR_2_REVERSE_OUTPUT_COMPARE,
            reverseTarget,
            acceleration
        );
    }

    return
        MOTOR_2_FORWARD_OUTPUT_COMPARE == forwardTarget
        && MOTOR_2_REVERSE_OUTPUT_COMPARE == reverseTarget;
}

/// \brief
///    Sets motor 1 rotation according to given switch press information.
///
/// This function stores desired motor state as static variables. Every time
/// this function is called, desired state is updated if switch press
/// information requires this. Otherwise, motor speed is increased by motor
/// acceleration value as stored in config header.
///
/// \param clockwiseEnd
///    If the switch in end of clockwise motion is pressed
///
/// \param counterClockwiseEnd
///    If the switch in end of counter-clockwise motion is pressed
///
/// \param uint8_t positionTarget
///    How far from clockwise end to go
void controlLinearMotor(
    bool clockwiseEnd,
    bool counterClockwiseEnd,
    uint8_t positionTarget
) {
    static uint8_t forwardTarget = MOTOR_1_SPEED;
    static uint8_t reverseTarget = 0x00;

    // For homing
    static uint8_t clockwiseEndReached = false;
    static uint8_t counter = 0;
    static bool homingComplete = false;

    // In clockwise end: change direction
    if(clockwiseEnd && !counterClockwiseEnd) {
        forwardTarget = MOTOR_1_SPEED;
        reverseTarget = 0x00;
    }
    // In counter-clockwise end: change direction
    else if(!clockwiseEnd && counterClockwiseEnd) {
        forwardTarget = 0x00;
        reverseTarget = MOTOR_1_SPEED;
    }
    // Should not happen: stop motor
    else if (clockwiseEnd && counterClockwiseEnd) {
        forwardTarget = 0x00;
        reverseTarget = 0x00;
    }

    switch (positionTarget) {
    case KEEP_MOVING:
        homingComplete = false;
        clockwiseEndReached = false;
        counter = 0;
        break;
    case HOLD:
        forwardTarget = 0x00;
        reverseTarget = 0x00;
        break;
    default:
        if (clockwiseEnd) {
            clockwiseEndReached = true;
        }
        else if (clockwiseEndReached) {
            if (counter < positionTarget) {
                counter++;
                break;
            }

           homingComplete = true;
        }
        break;
    }

    if (homingComplete) {
        updateMotor1Speed(0x00, 0x00, MOTOR_1_ACCELERATION);
        return;
    }

     updateMotor1Speed(forwardTarget, reverseTarget, MOTOR_1_ACCELERATION);
}

/// \brief
///    Sets motor 2 rotation following a predefined pattern.
///
/// \param forwardTarget
///    Forward target speed
///
/// \param reverseTarget
///    Reverse target speed
///
/// \param acceleration
///    Desired acceleration
///
/// \param retainTime
///    How long to retain current position after reached
///
/// \return
///    If speed has been reached and wait time is completed
bool controlRotaryMotor(
    uint8_t forwardTarget,
    uint8_t reverseTarget,
    uint8_t acceleration,
    uint8_t retainTime
) {
    static uint8_t counter = 0;
    static uint8_t reached = false;

    reached = updateMotor2Speed(forwardTarget, reverseTarget, acceleration);

    if (!reached) {
        return false;
    }

    if (counter < retainTime) {
        counter++;
        return false;
    }

    counter = 0;
    return true;
}

int main() {
    INDICATOR_DATA_DIR |= BV(INDICATOR_DATA_DIR_PIN);

    SWITCH_1_DATA |= BV(SWITCH_1_DATA_PIN);
    SWITCH_2_DATA |= BV(SWITCH_2_DATA_PIN);

    MOTOR_1_FORWARD_DATA_DIR |= BV(MOTOR_1_FORWARD_DATA_DIR_PIN);
    MOTOR_1_REVERSE_DATA_DIR |= BV(MOTOR_1_REVERSE_DATA_DIR_PIN);
    MOTOR_2_FORWARD_DATA_DIR |= BV(MOTOR_2_FORWARD_DATA_DIR_PIN);
    MOTOR_2_REVERSE_DATA_DIR |= BV(MOTOR_2_REVERSE_DATA_DIR_PIN);

    Atmega8::initializeTimer0(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );
    Atmega8::initializeTimer1(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );
    Atmega8::initializeTimer2(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );

    OCR1AH = 0x00;
    OCR1BH = 0x00;

    // Set non-inverting pwm
    TCCR0A |= BV(COM0A1) | BV(COM0B1);
    TCCR1A |= BV(COM1A1) | BV(COM1B1);
    TCCR2A |= BV(COM2A1) | BV(COM2B1);

    initializeMotors();

    uint32_t indicatorCounter = 0;
    uint32_t danceCounter = 0;
    uint8_t gradient = 0;

    uint8_t dancePosition = 0;

    while (true) {
        if (indicatorCounter % INDICATOR_HALF_PERIOD == 0) {
            toggleIndicator();
        }
        indicatorCounter++;

        bool switchState[2];
        readSwitches(switchState);

        uint8_t * current = danceSequence[dancePosition % DANCE_SEQUENCE_LENGTH];

        controlLinearMotor(switchState[0], switchState[1], current[4]);
        bool nextStep = controlRotaryMotor(
            current[0],
            current[1],
            current[2],
            current[3]
        );

        if (nextStep) {
            dancePosition++;
        }

        _delay_ms(LOOP_DELAY);
    }

}
